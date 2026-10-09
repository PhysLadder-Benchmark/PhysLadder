"""Manual fixture diagnostics, never imported by the task runner.

These checks deliberately set up fixtures or apply test drives. They do not
control a robot, do not call GPT and are NOT task-success evidence. They test
whether the declared contact geometry can support the required physical state.
"""

import math

import numpy as np


def _pose(scene, name, pose):
    body = scene.bodies[name]
    body.set_world_pose(np.array(pose[:3]), np.array(pose[3:]))
    body.set_linear_velocity(np.zeros(3))
    body.set_angular_velocity(np.zeros(3))


def _advance(scene, seconds):
    for _ in range(round(seconds / scene.config.control_dt_s)):
        scene.backend.step(scene.config.control_dt_s)


def run(scene):
    """Return measurements, failing if a claimed passive mechanism is absent."""
    from ..tasks.bench.spec import Joint

    task = scene.config.task_id
    if task == "L2-06":
        from .orange_probe import run as probe_orange

        return probe_orange(scene)
    if hasattr(scene, "spec") and "native_instances" in scene.spec.parameters:
        from .native_l2_probe import run as probe_native

        return probe_native(scene)
    result = {"task": task, "scope": "fixture-only, no robot policy or success score"}
    if task in {"L1-01", "L1-03", "L1-05", "L1-07", "L1-08", "L1-09"}:
        article = scene.design.articles[-1]
        goal = list(article.goal)
        goal[2] += 0.008
        if task == "L1-08":
            # Check a reachable release *inside* the mouth before fingers on
            # the narrower lower body would hit the other cup's rim. This is a
            # gravity/contact diagnostic, not a scripted robot grasp or success.
            lower = scene.measure()["lower_cup"]["pose"]
            p = scene.spec.parameters
            goal[:3] = [lower[0], lower[1], lower[2] + p["cup_height"] - p["entry_depth"]]
            result["release_entry_depth_m"] = p["entry_depth"]
        _pose(scene, article.name, goal)
        _advance(scene, 4 if task == "L1-09" else 2)
        row = scene.measure()[article.name]
        support = row["fixture_force_n"] + sum(row["pair_forces"].values())
        drift = float(np.linalg.norm(np.array(row["pose"][:2]) - goal[:2]))
        result.update(
            support_n=support, horizontal_drift_m=drift, final_pose=row["pose"]
        )
        from ..tasks.bench.evaluation import BenchEvaluator
        from ..tasks.bench.scoring_l1 import RULES

        # Inspect geometry criteria without inventing a grasp witness or success.
        _, criteria, _ = RULES[task](
            BenchEvaluator(scene.spec, scene.initial), scene.measure()
        )
        result["geometry_criteria"] = criteria
        key = {
            "L1-01": "inside_holder",
            "L1-03": "inside_socket",
            "L1-05": "inside_socket",
            "L1-07": "hook_through_head_opening",
            "L1-08": "nested",
            "L1-09": "stem_end_inside",
        }[task]
        if not criteria[key]:
            raise RuntimeError(
                "Physical resting pose rejected by geometry scorer: " + str(result)
            )
        if task == "L1-07" and not criteria["target_hook_supported"]:
            raise RuntimeError("Hat is not carried by the selected lowest peg: " + str(result))
        drift_limit = 0.02
        if task == "L1-09":
            # A rigid rose can lean until its tip rests at the cup's inner wall.
            drift_limit = scene.spec.parameters["radius"] + scene.spec.parameters["stem_radius"]
        if task == "L1-07":
            # A hanging visor naturally slides inward along a sloping peg and
            # pivots around its rear band. Bound that excursion by the actual peg
            # and visor span, not the old miniature J-hook's 8-cm heuristic.
            from ..tasks.l1.specs import local_corners

            peg_span = np.linalg.norm(np.array(scene.spec.parameters["hook_end"])
                                      - scene.spec.parameters["hook_start"])
            drift_limit = float(peg_span + np.linalg.norm(np.ptp(local_corners(article), axis=0)))
        result["horizontal_drift_limit_m"] = drift_limit
        if support < scene.spec.parameters.get("min_support_n", 0.05) or drift > drift_limit:
            raise RuntimeError("Open-cavity/hook support probe failed: " + str(result))
    elif task in {"L2-03", "L2-04", "L2-07"}:
        from .l2_prop_probe import run as probe_l2

        return probe_l2(scene)
    elif task == "L3-03":
        from pxr import Gf, UsdPhysics

        # Clamp the bottle only in this diagnostic. Cap is free to translate
        # along Z; bounded torque provides rotation, never commanded axial rise.
        scene.create_joint(
            Joint("probe_hold_bottle", "", "bottle", tuple(scene.initial["bottle"][:3]), {})
        )
        guide = UsdPhysics.Joint.Define(
            scene.stage, scene.root + "/Joints/probe_turn_cap"
        )
        guide.CreateBody1Rel().SetTargets([scene.root + "/cap"])
        guide.CreateLocalPos0Attr(Gf.Vec3f(*scene.initial["cap"][:3]))
        guide.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        # Native D6 unlimited rotation is its X twist, not Y/Z swing. Map
        # local +X onto world +Z. Only lateral translations are locked;
        # angular stabilization below uses bounded torques, without a twist limit.
        q = Gf.Quatf(2**-0.5, Gf.Vec3f(0, -(2**-0.5), 0))
        guide.CreateLocalRot0Attr(q)
        guide.CreateLocalRot1Attr(q)
        for axis in ("transY", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(guide.GetPrim(), axis)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)
        start = scene.measure()["cap"]["pose"][2]
        previous = scene.backend.before_physics_step
        turn = False
        integrated_error = 0.0

        def axial_load(dt):
            from ..robots.transforms import pose_transform

            nonlocal integrated_error
            previous(dt)
            body = scene.bodies["cap"]
            omega = body.get_angular_velocity()
            pos, quat = body.get_world_pose()
            normal = pose_transform([*pos, *quat])[:3, 2]
            torque = np.cross(normal, [0, 0, 1]) * 0.01 - omega * 0.0001
            error = 1.0 - omega[2]
            if turn:
                integrated_error = np.clip(integrated_error + error * dt, -20, 20)
            torque[2] = (
                np.clip(0.0005 * error + 0.0001 * integrated_error, -0.002, 0.002)
                if turn
                else 0
            )
            scene.pairs["cap"].apply_forces_and_torques_at_pos(
                forces=np.array([[0, 0, 0.25 if turn else 0.35]], dtype=np.float32),
                torques=np.array([torque], dtype=np.float32),
                is_global=True,
            )

        scene.backend.before_physics_step = axial_load
        _advance(scene, 1)
        pull_only_rise = scene.measure()["cap"]["pose"][2] - start
        result["pull_only_rise_m"] = float(pull_only_rise)
        if pull_only_rise > 0.004:
            raise RuntimeError("Cap pulled free without threading: " + str(result))
        turn = True
        samples = []
        rows = scene.measure()
        prev_angle = 2 * math.atan2(rows["cap"]["pose"][6], rows["cap"]["pose"][3])
        engaged_turn = 0.0
        disengaged = False
        for step in range(round(35 / scene.config.control_dt_s)):
            scene.backend.step(scene.config.control_dt_s)
            rows = scene.measure()
            r = rows["cap"]
            angle = 2 * math.atan2(r["pose"][6], r["pose"][3])
            delta = (angle - prev_angle + math.pi) % (2 * math.pi) - math.pi
            prev_angle = angle
            relative_z = r["pose"][2] - rows["bottle"]["pose"][2]
            if relative_z <= scene.spec.parameters["engaged_cap_z_max"]:
                engaged_turn += delta
            samples.append(
                {
                    "time_s": (step + 1) * scene.config.control_dt_s,
                    "z": r["pose"][2],
                    "pose": r["pose"],
                    "pair_force_n": r["pair_forces"]["bottle"],
                    "angular_velocity": r["angular_velocity"],
                    "engaged_turn_rad": float(engaged_turn),
                }
            )
            clearance = (
                relative_z
                - scene.spec.parameters["cap_half"]
                - scene.spec.parameters["neck_top"]
            )
            if clearance > 0.004 and r["pair_forces"]["bottle"] < 0.05:
                disengaged = True
                break
        scene.backend.before_physics_step = previous
        max_rise = max(r["z"] for r in samples) - start
        result.update(
            max_free_axis_rise_m=float(max_rise),
            axial_test_force_n=0.25,
            torque_cap_nm=0.002,
            engaged_turn_rad=float(engaged_turn),
            cap_fully_clear=disengaged,
            samples=samples,
        )
        if not disengaged or engaged_turn < 2 * math.pi:
            raise RuntimeError(
                "Thread did not complete rotation and disengagement: " + str(result)
            )
    else:
        result["scope"] = "initial physical scene check only; no separate fixture probe"
    result["passed"] = True
    return result
