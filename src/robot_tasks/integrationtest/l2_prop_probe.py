"""L2 contact/clearance diagnostics; fixture drives, never GPT task success."""

import numpy as np

from ..tasks.bench.spec import Joint


def advance(scene, seconds):
    for _ in range(round(seconds / scene.config.control_dt_s)):
        scene.backend.step(scene.config.control_dt_s)


def run(scene):
    task = scene.config.task_id
    result = {"task": task, "scope": "fixture-only contact diagnostic; no robot or model-success evidence"}
    if task == "L2-03":
        scene.create_joint(Joint("probe_book_pull", "", "middle_book", scene.initial["middle_book"][:3],
                                 {"transX": (-0.12, 0, 40, 4)}))
        drive = scene.drives["probe_book_pull", "transX"]
        drive.CreateMaxForceAttr(2.0)
        drive.CreateTargetPositionAttr(-0.10)
        advance(scene, 4)
        row = scene.measure()["middle_book"]
        p = scene.spec.parameters
        clearance = float(np.min((np.asarray(row["corners"]) - p["stack_front_world"]) @ p["outward_axis_world"]))
        result.update(rear_clearance_m=clearance, outward_axis_world=p["outward_axis_world"])
        if clearance < p["clearance_m"]:
            raise RuntimeError("Middle book cannot clear fixed neighbors: " + str(result))
    elif task == "L2-04":
        from .native_l2_probe import run as native_run

        return native_run(scene)
    elif task == "L2-07":
        drive = scene.drives["drawer_slide", "transX"]
        drive.CreateStiffnessAttr(80.0)
        drive.CreateDampingAttr(6.0)
        drive.CreateMaxForceAttr(2.0)
        for i in range(36):
            drive.CreateTargetPositionAttr(-0.12 * (i + 1) / 36)
            advance(scene, scene.config.control_dt_s)
        advance(scene, 1)
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.15)
        advance(scene, 2)
        row = scene.measure()["drawer"]
        travel = float((np.asarray(row["pose"][:3]) - scene.initial["drawer"][:3]) @ scene.spec.parameters["outward_axis_world"])
        result.update(outward_travel_m=travel, rail_support_n=row["fixture_force_n"])
        if not 0.10 <= travel <= 0.133 or row["fixture_force_n"] < 0.05:
            raise RuntimeError("Roofed drawer cannot open and remain supported: " + str(result))
    result["passed"] = True
    return result
