"""Native-asset mechanism checks, never policy baselines or GPT success runs."""

import numpy as np


def advance(scene, seconds):
    for _ in range(round(seconds / scene.config.control_dt_s)):
        scene.backend.step(scene.config.control_dt_s)


def run(scene):
    result = {
        "task": scene.config.task_id,
        "gpt_called": False,
        "scope": "mechanism-only diagnostic; direct test actuation, not robot task success",
    }
    if scene.config.task_id == "L2-07":
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(
            scene.stage.GetPrimAtPath(scene.root + "/organizer/Joints/Drawer1"),
            "linear",
        )
        old = {
            k: getattr(drive, "Get" + k + "Attr")().Get()
            for k in ("Stiffness", "Damping", "MaxForce", "TargetPosition")
        }
        start = scene.measure()["drawer"]["pose"][:3]
        try:
            drive.CreateStiffnessAttr(120.0)
            drive.CreateDampingAttr(6.0)
            drive.CreateMaxForceAttr(4.0)
            drive.CreateTargetPositionAttr(-0.085)
            advance(scene, 3)
        finally:
            for name, value in old.items():
                getattr(drive, "Create" + name + "Attr")(value)
        advance(scene, 1)
        final = scene.measure()["drawer"]
        axis = np.asarray(scene.spec.parameters["outward_axis_world"])
        travel = float((np.asarray(final["pose"][:3]) - start) @ axis)
        lateral = np.asarray(final["pose"][:3]) - start - travel * axis
        result.update(
            opening_m=travel,
            lateral_drift_m=float(np.linalg.norm(lateral)),
            released_speed_m_s=float(np.linalg.norm(final["linear_velocity"])),
        )
        if not 0.075 <= travel <= 0.093 or np.linalg.norm(lateral) > 0.002:
            raise RuntimeError(
                "Native drawer rail failed opening/release diagnostic: " + str(result)
            )
    elif scene.config.task_id == "L2-04":
        from pxr import Gf, UsdPhysics

        from ..tasks.bench.evaluation import relative
        from ..tasks.bench.scoring_l2 import ball_contained

        def count_inside():
            rows = scene.measure()
            return sum(
                ball_contained(
                    relative(rows[n], rows["bowl"])[:3, 3], scene.spec.parameters
                )
                for n in scene.load_names
            )

        start = scene.measure()["bowl"]["pose"][:3]
        joint = UsdPhysics.Joint.Define(
            scene.stage, scene.root + "/DiagnosticTransport"
        )
        joint.CreateBody1Rel().SetTargets([scene.paths["bowl"]])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*start))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            low, high = {"transY": (-0.18, 0.01), "transZ": (-0.01, 0.06)}.get(
                axis, (1.0, -1.0)
            )
            limit.CreateLowAttr(low)
            limit.CreateHighAttr(high)
        drives = {}
        for axis in ("transY", "transZ"):
            d = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            d.CreateStiffnessAttr(700.0)
            d.CreateDampingAttr(25.0)
            d.CreateMaxForceAttr(20.0)
            d.CreateTargetPositionAttr(0.0)
            drives[axis] = d
        counts = [count_inside()]
        travel = scene.design.articles[0].goal[1] - scene.design.articles[0].initial[1]
        for axis, begin, end, seconds in (
            ("transZ", 0.0, 0.04, 2.0),
            ("transY", 0.0, travel, 4.0),
            ("transZ", 0.04, 0.0, 2.0),
        ):
            steps = round(seconds / scene.config.control_dt_s)
            for i in range(steps):
                t = (i + 1) / steps
                drives[axis].CreateTargetPositionAttr(
                    begin + (end - begin) * t * t * (3 - 2 * t)
                )
                scene.backend.step(scene.config.control_dt_s)
            counts.append(count_inside())
        joint.CreateJointEnabledAttr(False)
        advance(scene, 0.5)
        counts.append(count_inside())
        result.update(contained_counts=counts, required=scene.spec.parameters["count"])
        if min(counts) != scene.spec.parameters["count"]:
            raise RuntimeError(
                "Native loaded bowl lost contents during gentle diagnostic transport: "
                + str(result)
            )
    elif scene.config.task_id == "L2-01":
        from pxr import Gf, UsdPhysics

        from ..robots.transforms import pose_transform

        rows = scene.measure()
        for name in ("body", "cap"):
            p = rows[name]["pose"]
            cls = UsdPhysics.FixedJoint if name == "body" else UsdPhysics.PrismaticJoint
            joint = cls.Define(scene.stage, scene.root + "/Diagnostic_" + name)
            joint.CreateBody1Rel().SetTargets([scene.paths[name]])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*p[:3]))
            joint.CreateLocalRot0Attr(Gf.Quatf(p[3], Gf.Vec3f(*p[4:])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
            if name == "cap":
                joint.CreateAxisAttr("Z")
                joint.CreateLowerLimitAttr(0.0)
                joint.CreateUpperLimitAttr(0.065)
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
                drive.CreateStiffnessAttr(100.0)
                drive.CreateDampingAttr(0.5)
                drive.CreateMaxForceAttr(4.0)
                drive.CreateTargetPositionAttr(0.05)
        advance(scene, 3)
        rows = scene.measure()
        b, c = (pose_transform(rows[n]["pose"]) for n in ("body", "cap"))
        tip = np.linalg.inv(c) @ b @ np.array([0.0, 0.0, 0.128, 1.0])
        gap = 0.105 - float(tip[2])
        result.update(
            axial_clearance_m=gap,
            test_force_limit_n=4.0,
            fixture="Fixed barrel and axially guided cap in diagnostic only",
        )
        if gap < 0.002:
            raise RuntimeError(
                "Native cap fit did not release under diagnostic pull: " + str(result)
            )
    elif scene.config.task_id == "L2-03":
        from pxr import Gf, UsdPhysics

        start = scene.measure()["middle_book"]["pose"][:3]
        joint = UsdPhysics.PrismaticJoint.Define(
            scene.stage, scene.root + "/DiagnosticPull"
        )
        joint.CreateBody1Rel().SetTargets([scene.paths["middle_book"]])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*start))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        joint.CreateAxisAttr("Y")
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(0.18)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateStiffnessAttr(150.0)
        drive.CreateDampingAttr(8.0)
        drive.CreateMaxForceAttr(8.0)
        drive.CreateTargetPositionAttr(0.15)
        try:
            advance(scene, 4.0)
            row = scene.measure()["middle_book"]
            p = scene.spec.parameters
            clearance = float(
                np.min(
                    (np.asarray(row["corners"]) - p["stack_front_world"])
                    @ p["outward_axis_world"]
                )
            )
            result.update(
                rear_clearance_m=clearance,
                travel_m=float(
                    np.asarray(row["pose"][:3]) @ p["outward_axis_world"]
                    - np.asarray(start) @ p["outward_axis_world"]
                ),
            )
            if clearance < p["clearance_m"]:
                raise RuntimeError(
                    "Native book cannot clear fixed neighbours: " + str(result)
                )
        finally:
            joint.CreateJointEnabledAttr(False)
    else:
        raise ValueError("No native mechanism diagnostic for " + scene.config.task_id)
    result["passed"] = True
    return result
