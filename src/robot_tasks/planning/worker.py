"""GPU worker. The same planner handles every task and both R5 arms."""

import argparse
import json
import os
from pathlib import Path
import sys

# Keep CUDA/JIT artifacts with the installed research environment, not the OS disk.
_cache = Path(sys.prefix) / "cache"

import numpy as np
from .assets import TCP_FRAME, build_assets
from ..robots.profile import load_robot_profile
from ..robots.control import bounded_measured_joints
from ..robots.transforms import (
    pose_transform,
    transform_pose,
    axis_rotation,
)
from .trajectory import JointPath


def link_transforms(kin, positions):
    result = {kin.profile.base_link: np.eye(4)}
    transform = np.eye(4)
    for j, origin, q in zip(kin.chain, kin.origins, positions):
        rotation = np.eye(4)
        rotation[:3, :3] = axis_rotation(j.axis, q)
        transform = transform @ origin @ rotation
        result[j.child] = transform.copy()
    return result


class CuroboWorker:
    def __init__(self, profile, directory):
        for key, leaf in [
            ("XDG_CACHE_HOME", "xdg"),
            ("TORCH_EXTENSIONS_DIR", "torch"),
            ("WARP_CACHE_PATH", "warp"),
        ]:
            os.environ.setdefault(key, str(_cache / leaf))
        import torch
        import curobo.runtime
        import curobo._src.runtime

        # This pinned release copies scalar settings in its public re-export.
        # Set both before constructing compilers/planners.
        curobo.runtime.cache_dir = str(_cache / "curobo")
        curobo._src.runtime.cache_dir = curobo.runtime.cache_dir
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

        self.torch = torch
        self.assets, self.kin = build_assets(profile, directory)
        self.profile = profile
        self.planner = MotionPlanner(
            MotionPlannerCfg.create(
                robot=self.assets["config"],
                scene_model={"cuboid": {}},
                collision_cache={"obb": 256, "mesh": 128},
                position_tolerance=0.001,
                orientation_tolerance=0.01,
                optimizer_collision_activation_distance=0.002,
                interpolation_dt=0.025,
                num_ik_seeds=32,
                num_trajopt_seeds=4,
                random_seed=0,
            )
        )
        self.metadata = self.assets["metadata"]
        import hashlib
        import importlib.metadata

        self.metadata.update(
            python=sys.executable,
            platform=sys.platform,
            library_version=importlib.metadata.version("nvidia-curobo"),
            torch_version=torch.__version__,
            gpu=torch.cuda.get_device_name(0),
            collision_geometry_sha256=hashlib.sha256(
                json.dumps(
                    {key: self.assets[key] for key in ("spheres", "finger_geometry")},
                    sort_keys=True,
                ).encode()
            ).hexdigest(),
        )
        self.metadata.update(
            collision_scope="table, static fixtures, other R5 arm; held-object clearance guard",
            payload_source="simulator contacts and surface geometry; planner-only privilege",
        )
        # Cross-check the generated model before allowing any real actuation.
        q = torch.tensor(
            [profile.mounts[0].home_positions_rad], device="cuda", dtype=torch.float32
        )
        pose = self.planner.kinematics.get_link_poses(q, [TCP_FRAME])
        actual = np.r_[
            pose.position.detach().cpu().numpy().reshape(-1, 3)[0],
            pose.quaternion.detach().cpu().numpy().reshape(-1, 4)[0],
        ]
        from ..robots.transforms import pose_errors

        if (
            max(
                pose_errors(
                    actual, self.kin.forward(q[0].cpu().numpy(), "left", "robot_base")
                )
            )
            > 1e-5
        ):
            raise RuntimeError("cuRobo FK differs from R5 URDF/TCP")

    def spheres(self, state):
        rows = {k: list(v) for k, v in self.assets["spheres"].items()}
        end = list(self.assets["link6_static"])
        for (_, geometry), q in zip(
            self.assets["finger_geometry"].items(), state["finger_positions_m"]
        ):
            transform = np.array(geometry["origin"])
            transform[:3, 3] += transform[:3, :3] @ (np.array(geometry["axis"]) * q)
            for s in geometry["spheres"]:
                end.append(
                    {
                        "center": (
                            transform[:3, :3] @ s["center"] + transform[:3, 3]
                        ).tolist(),
                        "radius": s["radius"],
                    }
                )
        rows[self.profile.ee_link] = end
        return rows

    def world_spheres(self, arm, q, spheres):
        links = link_transforms(self.kin, q)
        result = []
        for link, rows in spheres.items():
            t = self.kin.bases[arm] @ links[link]
            for s in rows:
                if s["radius"] > 0:
                    result.append([*(t[:3, :3] @ s["center"] + t[:3, 3]), s["radius"]])
        return np.asarray(result)

    def world(self, arm, request):
        from copy import deepcopy
        from curobo.scene import Scene

        obstacles = deepcopy(request["scene"]["obstacles"])
        inv = np.linalg.inv(self.kin.bases[arm])
        for category in ("cuboid", "mesh"):
            for row in obstacles.get(category, {}).values():
                row["pose"] = list(transform_pose(inv @ pose_transform(row["pose"])))
        # Conservative boxes around the other arm's collision spheres. Named
        # meshes cache their vertices, so a moving mesh must never reuse a stale
        # geometry name; OBB poses can be updated safely within CUDA graph caches.
        for other, state in request["states"].items():
            if other == arm:
                continue
            rows = self.world_spheres(
                other, state["joint_positions_rad"], self.spheres(state)
            )
            for index, (x, y, z, r) in enumerate(rows):
                pose = np.eye(4)
                pose[:3, 3] = [x, y, z]
                obstacles.setdefault("cuboid", {})[f"other_{other}_{index}"] = {
                    "pose": list(transform_pose(inv @ pose)),
                    "dims": [2 * r] * 3,
                }
        return Scene.create(obstacles)

    def plan(self, request):
        from curobo.types import JointState, GoalToolPose
        from ..robots.transforms import pose_errors

        torch = self.torch
        paths, diagnostics = {}, {}
        states = request["states"]
        for command in request["commands"]:
            arm = command["arm"]
            state = states[arm]
            bounds = [
                next(x for x in self.profile.operating_limits if x.joint == n)
                for n in self.profile.joint_names
            ]
            # PhysX can report a few float32 ULPs beyond a rail limit. Keep the
            # raw observation, but supply a strictly bounded planning reference.
            reference = bounded_measured_joints(state["joint_positions_rad"], bounds)
            rows = self.spheres(state)[self.profile.ee_link]
            spheres = torch.tensor(
                [[*s["center"], s["radius"]] for s in rows],
                device="cuda",
                dtype=torch.float32,
            )
            self.planner.kinematics.config.kinematics_config.update_link_spheres(
                self.profile.ee_link, spheres
            )
            self.planner.update_world(self.world(arm, request))
            q0 = torch.tensor(
                [reference], device="cuda", dtype=torch.float32
            )
            start = JointState.from_position(
                q0, joint_names=list(self.profile.joint_names)
            )
            if command["kind"] == "cartesian_pose":
                world = self.kin.world_pose(command["values"], arm, command["frame"])
                local = transform_pose(
                    np.linalg.inv(self.kin.bases[arm]) @ pose_transform(world)
                )
                goal = GoalToolPose(
                    tool_frames=[TCP_FRAME],
                    position=torch.tensor(
                        local[:3], device="cuda", dtype=torch.float32
                    ).reshape(1, 1, 1, 1, 3),
                    quaternion=torch.tensor(
                        local[3:], device="cuda", dtype=torch.float32
                    ).reshape(1, 1, 1, 1, 4),
                )
                result = self.planner.plan_pose(goal, start, max_attempts=3)
            elif command["kind"] == "joint_position":
                goal = JointState.from_position(
                    torch.tensor(
                        [command["values"]], device="cuda", dtype=torch.float32
                    ),
                    joint_names=list(self.profile.joint_names),
                )
                result = self.planner.plan_cspace(goal, start, max_attempts=3)
            else:
                raise ValueError("Unsupported motion channel")
            if result is None or not bool(result.success.any()):
                raise ValueError(
                    f"{arm}: no valid collision-checked trajectory; choose a different target or view"
                )
            trajectory = result.get_interpolated_plan()
            q = (
                trajectory.position.detach()
                .cpu()
                .numpy()
                .reshape(-1, len(trajectory.joint_names))
            )
            indices = [
                trajectory.joint_names.index(n) for n in self.profile.joint_names
            ]
            q = q[:, indices]
            path = JointPath.from_waypoints(q, state["joint_positions_rad"], bounds)
            if command["kind"] == "cartesian_pose":
                error = pose_errors(self.kin.forward(path.positions[-1], arm), world)
                if error[0] > 0.002 or error[1] > 0.02:
                    raise ValueError(f"{arm}: endpoint FK residual exceeds tolerance")
            paths[arm] = {"positions": path.positions, "times_s": path.times_s}
            diagnostics[arm] = {
                "waypoints": len(path.positions),
                "duration_s": path.duration_s,
                "collision_checked": True,
                "payload_guard": "no held object",
                "start_joint_limit_correction_max_rad": max(
                    abs(q - r) for q, r in zip(state["joint_positions_rad"], reference)
                ),
            }
            self.check_payload(arm, path, request, diagnostics[arm])
        if len(paths) > 1:
            self.check_bimanual(paths, states)
        return {"id": request["id"], "paths": paths, "diagnostics": diagnostics}

    def check_bimanual(self, paths, states):
        names = list(paths)
        trajectories = {
            a: JointPath(tuple(paths[a]["positions"]), tuple(paths[a]["times_s"]))
            for a in names
        }
        geometry = {a: self.spheres(states[a]) for a in names}
        duration = max(t.duration_s for t in trajectories.values())
        for t in np.arange(0, duration + 0.01, 0.01):
            first, second = [
                self.world_spheres(a, trajectories[a].sample(t), geometry[a])
                for a in names
            ]
            distances = (
                np.linalg.norm(first[:, None, :3] - second[None, :, :3], axis=-1)
                - first[:, None, 3]
                - second[None, :, 3]
            )
            if np.any(distances < 0):
                raise ValueError(
                    "Bimanual planned trajectories overlap; neither arm was commanded"
                )

    def check_payload(self, arm, path, request, diagnostic):
        payloads = request["scene"].get("payloads", {}).get(arm, [])
        if not payloads:
            return
        import trimesh

        obstacles = request["scene"]["obstacles"]
        meshes = []
        for data in obstacles.get("mesh", {}).values():
            mesh = trimesh.Trimesh(
                vertices=data["vertices"], faces=data["faces"], process=True
            )
            mesh.apply_transform(pose_transform(data["pose"]))
            meshes.append(mesh)
        start = np.linalg.inv(
            pose_transform(
                self.kin.forward(request["states"][arm]["joint_positions_rad"], arm)
            )
        )
        for payload in payloads:
            # Measured held-body samples, never a fixed joint or object pose write.
            points = np.asarray(payload["surface_points_world"])
            local = points @ start[:3, :3].T + start[:3, 3]
            for t in np.arange(0, path.duration_s + 0.02, 0.02):
                pose = pose_transform(self.kin.forward(path.sample(t), arm))
                world = local @ pose[:3, :3].T + pose[:3, 3]
                if world[:, 2].min() < -0.002:
                    raise ValueError(
                        f"{arm}: carried-object clearance intersects table; lift before transport"
                    )
                for mesh in meshes:
                    # Negative outside; allow up to 1 mm contact tolerance, including final seating.
                    if np.any(trimesh.proximity.signed_distance(mesh, world) > 0.001):
                        raise ValueError(
                            f"{arm}: carried-object path penetrates fixture; choose clearance then approach"
                        )
        diagnostic["payload_guard"] = (
            "held surface samples checked against table and fixture (1 mm contact tolerance)"
        )


def main():
    import queue
    import threading

    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    incoming = queue.Queue()

    def read_requests():
        for line in sys.stdin:
            incoming.put(line)
        # Pipe ownership is the lifetime contract. In particular, killing the
        # Windows parent must not strand a WSL GPU process inside a long solve.
        os._exit(0)

    threading.Thread(target=read_requests, daemon=True).start()

    def emit(data):
        print("R5_PLANNER " + json.dumps(data, allow_nan=False), flush=True)

    try:
        worker = CuroboWorker(load_robot_profile(args.profile), args.directory)
        emit({"ready": True, **worker.metadata})
    except Exception as exc:
        import traceback

        traceback.print_exc()
        emit({"ready": False, "error": f"{type(exc).__name__}: {exc}"})
        return 2
    while True:
        line = incoming.get()
        request = {}
        try:
            request = json.loads(line)
            emit(worker.plan(request))
        except Exception as exc:
            import traceback

            traceback.print_exc()
            emit({"id": request.get("id"), "error": f"{type(exc).__name__}: {exc}"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
