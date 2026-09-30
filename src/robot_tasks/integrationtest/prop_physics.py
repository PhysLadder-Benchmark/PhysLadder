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
    result = {"task": task, "scope": "fixture-only, no robot policy or success score"}
    if task in {"L1-01", "L1-03", "L1-05", "L1-07", "L1-08"}:
        article = scene.design.articles[-1]
        goal = list(article.goal)
        goal[2] += 0.008
        _pose(scene, article.name, goal)
        _advance(scene, 2)
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
            "L1-07": "hook_through_handle",
            "L1-08": "nested",
        }[task]
        if not criteria[key]:
            raise RuntimeError(
                "Physical resting pose rejected by geometry scorer: " + str(result)
            )
        if support < 0.05 or drift > (0.08 if task == "L1-07" else 0.02):
            raise RuntimeError("Open-cavity/hook support probe failed: " + str(result))
    elif task == "L1-09":
        from ..tasks.bench.evaluation import BenchEvaluator
        from ..tasks.bench.scoring_l1 import flower

        # A released candidate placement must be physically supportable before
        # assessing the separate end-held sag experiment below.
        for i, a in enumerate(scene.design.articles):
            _pose(scene, a.name, [0.36, 0.20, 0.018 + i * 0.028, 1, 0, 0, 0])
        _advance(scene, 3)
        placed = scene.measure()
        _, criteria, _ = flower(BenchEvaluator(scene.spec, scene.initial), placed)
        result["placement_criteria"] = criteria
        result["placement_head_z"] = placed["flower_head"]["pose"][2]
        if not (criteria["stem_end_inside"] and criteria["flower_clear_of_table"]):
            raise RuntimeError(
                "Flexible flower placement cannot remain supported: " + str(result)
            )
        # Hold only the remote stem end horizontally above the bench. Sag must
        # follow gravity and passive joints, not an animation or pose write.
        # Keep the fixture diagnostic clear of either robot's prepared pose.
        # This offset is not used by the actual task or reset.
        test_height = 0.90
        for a in scene.design.articles:
            p = list(a.initial)
            p[2] += test_height
            _pose(scene, a.name, p)
        anchor = scene.design.articles[0].initial[:3]
        scene.create_joint(Joint("probe_fixed_tip", "", "stem_0", anchor, {}))
        # World joint was authored relative to the original pose; move its
        # world anchor up with the diagnostic assembly.
        from pxr import Gf, UsdPhysics

        joint = UsdPhysics.Joint.Get(
            scene.stage, scene.root + "/Joints/probe_fixed_tip"
        )
        joint.CreateLocalPos0Attr(
            Gf.Vec3f(anchor[0], anchor[1], anchor[2] + test_height)
        )
        _advance(scene, 3)
        row = scene.measure()["flower_head"]
        sag = anchor[2] + test_height - row["pose"][2]
        result.update(head_sag_m=float(sag))
        if sag < 0.04:
            raise RuntimeError(
                "Stem proxy is not sufficiently flexible: " + str(result)
            )
    elif task == "L2-04":
        from pxr import Gf, UsdPhysics

        from ..tasks.bench.evaluation import relative

        _pose(scene, "bowl", [0.40, 0, 0.13, 1, 0, 0, 0])
        scene.create_joint(
            Joint("probe_bowl_holder", "", "bowl", (0.37, 0.21, 0.001), {})
        )
        joint = UsdPhysics.Joint.Get(
            scene.stage, scene.root + "/Joints/probe_bowl_holder"
        )
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.40, 0, 0.13))
        scene.feed_armed = True  # Explicit test fixture trigger, not a model event.
        _advance(scene, 5)
        rows = scene.measure()
        positions = [
            relative(rows[f"load_{i}"], rows["bowl"])[:3, 3].tolist() for i in range(6)
        ]
        inside = all(
            np.linalg.norm(p[:2]) < 0.052 and 0.01 <= p[2] <= 0.05 for p in positions
        )
        result.update(
            load_positions_in_bowl=positions, feeder_gate_x=rows["feed_gate"]["pose"][0]
        )
        if not inside:
            raise RuntimeError(
                "Standardized feeder spilled outside receiving bowl: " + str(result)
            )
    elif task == "L2-05":
        from ..robots.transforms import pose_transform

        motor = scene.drives["lid_hinge", "rotY"]
        motor.CreateTypeAttr("force")
        motor.CreateStiffnessAttr(0.10)
        motor.CreateDampingAttr(0.01)
        motor.CreateMaxForceAttr(0.12)
        motor.CreateTargetPositionAttr(-80.0)

        def angle():
            r = pose_transform(scene.measure()["lid"]["pose"])
            return float(np.arctan2(r[2, 0], r[0, 0]))

        _advance(scene, 2)
        locked_angle = angle()
        # Relieve upward latch load before pulling its slider, as a person
        # would. This prevents the test fixture itself from wedging the tooth.
        motor.CreateTargetPositionAttr(0.0)
        _advance(scene, 1)
        slider = scene.drives["latch_spring", "transX"]
        slider.CreateTargetPositionAttr(0.018)
        _advance(scene, 1)
        motor.CreateTargetPositionAttr(-80.0)
        _advance(scene, 3)
        opened_angle = angle()
        from ..tasks.bench.evaluation import relative

        opened_rows = scene.measure()
        extension = float(
            relative(opened_rows["latch"], opened_rows["lid"])[0, 3] - 0.096
        )
        motor.CreateTargetPositionAttr(0.0)
        _advance(scene, 3)
        slider.CreateTargetPositionAttr(0.0)
        _advance(scene, 2)
        result.update(
            locked_angle_rad=locked_angle,
            open_angle_rad=opened_angle,
            latch_extension_m=extension,
            reclosed_angle_rad=angle(),
        )
        if locked_angle > 0.2 or opened_angle < 0.65 or abs(angle()) > 0.1:
            raise RuntimeError(
                "Physical latch did not block, unlock and reclose: " + str(result)
            )
    elif task == "L2-07":
        from pxr import UsdPhysics

        joint = UsdPhysics.Joint.Get(scene.stage, scene.root + "/Joints/drawer_slide")
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "transX")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(30.0)
        drive.CreateDampingAttr(4.0)
        drive.CreateTargetPositionAttr(0.11)
        _advance(scene, 4)
        opened = scene.measure()["drawer"]["pose"][0] - scene.initial["drawer"][0]
        drive.CreateTargetPositionAttr(0.0)
        _advance(scene, 4)
        row = scene.measure()["drawer"]
        closed = row["pose"][0] - scene.initial["drawer"][0]
        result.update(
            opening_m=float(opened),
            closing_error_m=float(closed),
            rail_support_n=row["fixture_force_n"],
        )
        if opened < 0.08 or abs(closed) > 0.02 or row["fixture_force_n"] < 0.05:
            raise RuntimeError("Drawer rail probe failed: " + str(result))
    elif task == "L3-03":
        from pxr import Gf, UsdPhysics

        # Clamp the bottle only in this diagnostic. Cap is free to translate
        # along Z; bounded torque provides rotation, never commanded axial rise.
        scene.create_joint(
            Joint("probe_hold_bottle", "", "bottle", (0.36, 0, 0.001), {})
        )
        guide = UsdPhysics.Joint.Define(
            scene.stage, scene.root + "/Joints/probe_turn_cap"
        )
        guide.CreateBody1Rel().SetTargets([scene.root + "/cap"])
        guide.CreateLocalPos0Attr(Gf.Vec3f(0.36, 0, 0.131))
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
