"""Task-independent, bounded Cartesian execution against the known bench.

No object pose, grasp target or recovery policy enters this module. Rejection
leaves the whole action unapplied; the caller must choose another target.
"""

from dataclasses import dataclass
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import IKError
from .transforms import axis_rotation, origin_transform, pose_transform
from .urdf import _mesh_path, read_urdf
from ..backends.scene_geometry import BENCH_POSITION, BENCH_SIZE, BENCH_TOP

VERSION = "cartesian_linear_v1"
LINEAR_STEP_M = 0.01
ANGULAR_STEP_RAD = math.radians(5)
LINEAR_SPEED_M_S = 0.10
ANGULAR_SPEED_RAD_S = 0.5
MAX_JOINT_STEP_RAD = 0.20
TRACKING_WINDOW_RAD = 0.08
TABLE_PENETRATION_M = 0.002


def interpolated_poses(start, goal):
    a, b = np.asarray(start, float), np.asarray(goal, float)
    pose_transform(a)
    pose_transform(b)
    qa, qb = a[3:] / np.linalg.norm(a[3:]), b[3:] / np.linalg.norm(b[3:])
    dot = float(qa @ qb)
    if dot < 0:
        qb, dot = -qb, -dot
    theta = math.acos(float(np.clip(dot, 0, 1)))
    distance, angle = float(np.linalg.norm(b[:3] - a[:3])), 2 * theta
    count = max(1, math.ceil(distance / LINEAR_STEP_M), math.ceil(angle / ANGULAR_STEP_RAD))
    if count > 512:
        raise ValueError("Cartesian path is too long; choose a closer target")
    result = []
    for fraction in np.linspace(0, 1, count + 1):
        q = (
            (1 - fraction) * qa + fraction * qb
            if theta < 1e-6
            else (math.sin((1 - fraction) * theta) * qa + math.sin(fraction * theta) * qb) / math.sin(theta)
        )
        result.append(tuple(np.r_[a[:3] + fraction * (b[:3] - a[:3]), q / np.linalg.norm(q)]))
    return result, distance, angle


class BenchGuard:
    """Robot collision-mesh envelopes versus the fixed tabletop, in metres.

    Bounding boxes are conservative (may reject a feasible pose). They also
    cover convex-decomposition collision geometry. No task obstacles are read.
    """

    def __init__(self, profile):
        from itertools import product

        self.profile = profile
        self.joints = read_urdf(profile.asset_path, dict(profile.package_roots)).joints
        self.bounds = {}
        for link in ET.parse(profile.asset_path).getroot().findall("link"):
            pieces = []
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    raise ValueError("Bench guard requires the pinned R5 STL collision meshes")
                path = _mesh_path(mesh.get("filename"), profile.asset_path, dict(profile.package_roots))
                data = path.read_bytes()
                count = struct.unpack_from("<I", data, 80)[0]
                if len(data) != 84 + 50 * count:
                    raise ValueError(f"Expected binary STL collision mesh: {path.name}")
                dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
                vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3)
                scale = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
                vertices = vertices * scale
                corners = np.array(list(product(*zip(vertices.min(axis=0), vertices.max(axis=0)))))
                origin = collision.find("origin")
                xyz = (0, 0, 0) if origin is None else tuple(map(float, origin.get("xyz", "0 0 0").split()))
                rpy = (0, 0, 0) if origin is None else tuple(map(float, origin.get("rpy", "0 0 0").split()))
                t = origin_transform(xyz, rpy)
                pieces.extend(corners @ t[:3, :3].T + t[:3, 3])
            self.bounds[link.get("name")] = np.asarray(pieces)

    def check(self, arm, positions, fingers):
        profile = self.profile
        mount = next(m for m in profile.mounts if m.name == arm)
        transforms = {profile.base_link: pose_transform(mount.base_pose_wxyz)}
        values = dict(zip(profile.joint_names + profile.gripper.joint_names, (*positions, *fingers)))
        remaining = list(self.joints)
        while remaining:
            ready = [j for j in remaining if j.parent in transforms]
            if not ready:
                raise ValueError("Disconnected R5 collision chain")
            for joint in ready:
                t = transforms[joint.parent] @ origin_transform(joint.origin_xyz, joint.origin_rpy)
                motion = np.eye(4)
                if joint.kind in {"revolute", "continuous"}:
                    motion[:3, :3] = axis_rotation(joint.axis, values[joint.name])
                elif joint.kind == "prismatic":
                    motion[:3, 3] = np.asarray(joint.axis) * values[joint.name]
                elif joint.kind != "fixed":
                    raise ValueError("Unsupported R5 collision joint")
                transforms[joint.child] = t @ motion
                remaining.remove(joint)
        for name, vertices in self.bounds.items():
            if name == profile.base_link or not len(vertices):
                continue
            t = transforms[name]
            world = vertices @ t[:3, :3].T + t[:3, 3]
            lo, hi = world.min(axis=0), world.max(axis=0)
            overlap = all(
                hi[i] > BENCH_POSITION[i] - BENCH_SIZE[i] / 2
                and lo[i] < BENCH_POSITION[i] + BENCH_SIZE[i] / 2
                for i in (0, 1)
            )
            if overlap and lo[2] < BENCH_TOP - TABLE_PENETRATION_M:
                raise ValueError(
                    f"table_collision: {name} envelope reaches z={lo[2]:.4f} m; choose another pose/path"
                )


@dataclass
class JointPath:
    points: np.ndarray
    times: np.ndarray
    elapsed: float = 0.0

    @property
    def finished(self):
        return self.elapsed >= self.times[-1]

    def advance(self, dt, measured):
        current = np.array([np.interp(self.elapsed, self.times, self.points[:, i]) for i in range(6)])
        # Do not run the reference farther ahead if physics cannot track it.
        if np.max(abs(current - measured)) <= TRACKING_WINDOW_RAD:
            self.elapsed = min(float(self.times[-1]), self.elapsed + dt)
        return tuple(float(np.interp(self.elapsed, self.times, self.points[:, i])) for i in range(6))


def plan_cartesian(kinematics, guard, arm, seed, goal, fingers, *, budget_s):
    poses, distance, angle = interpolated_poses(kinematics.forward(seed, arm), goal)
    points = [np.asarray(seed, float)]
    times = [0.0]
    limits = {b.joint: b.velocity for b in kinematics.profile.operating_limits}
    velocity = np.array([limits[j] for j in kinematics.profile.joint_names])
    base_dt = max(distance / LINEAR_SPEED_M_S, angle / ANGULAR_SPEED_RAD_S) / (len(poses) - 1)
    guard.check(arm, points[0], fingers)
    for pose in poses[1:]:
        solution = kinematics.solve(pose, arm, points[-1])
        if not solution.converged:
            raise IKError(solution)
        q = np.array(solution.positions_rad)
        change = abs(q - points[-1])
        if max(change) > MAX_JOINT_STEP_RAD:
            raise ValueError("joint_branch_jump: choose a smaller motion or a different orientation")
        # Check the joint interpolation between IK samples, not just the endpoints.
        for f in np.linspace(0, 1, max(2, math.ceil(max(change) / 0.02) + 1))[1:]:
            guard.check(arm, points[-1] + f * (q - points[-1]), fingers)
        times.append(times[-1] + max(base_dt, float(max(change / velocity)), 1e-3))
        points.append(q)
    if times[-1] > budget_s:
        raise ValueError(
            "motion_budget: path exceeds action budget at configured speed; choose a closer target"
        )
    return JointPath(np.array(points), np.array(times))
