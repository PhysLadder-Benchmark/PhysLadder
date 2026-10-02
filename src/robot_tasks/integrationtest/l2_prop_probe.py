"""L2 contact/clearance diagnostics; fixture drives, never GPT task success."""

import math

import numpy as np

from ..robots.transforms import pose_transform
from ..tasks.bench.evaluation import relative
from ..tasks.bench.scoring_l2 import ball_contained
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
        p = scene.spec.parameters

        def contents():
            rows = scene.measure()
            return [ball_contained(relative(rows[f"load_{i}"], rows["bowl"])[:3, 3], p)
                    for i in range(p["count"])]

        initial_count = sum(contents())
        if initial_count != p["count"]:
            raise RuntimeError(f"Initial bowl packing spilled: {initial_count}/{p['count']}")
        scene.create_joint(Joint("probe_bowl_transport", "", "bowl", scene.initial["bowl"][:3],
                                 {"transY": (-0.24, 0, 250, 18), "transZ": (0, 0.06, 500, 25)}))
        vertical = scene.drives["probe_bowl_transport", "transZ"]
        horizontal = scene.drives["probe_bowl_transport", "transY"]
        for i in range(24):
            vertical.CreateTargetPositionAttr(0.050 * (i + 1) / 24)
            advance(scene, scene.config.control_dt_s)
        for i in range(60):
            horizontal.CreateTargetPositionAttr(-0.24 * (i + 1) / 60)
            advance(scene, scene.config.control_dt_s)
        advance(scene, 2)
        transported = sum(contents())
        for i in range(24):
            vertical.CreateTargetPositionAttr(0.050 * (1 - (i + 1) / 24))
            advance(scene, scene.config.control_dt_s)
        advance(scene, 2)
        final_count = sum(contents())
        result.update(initial_contained=initial_count, transported_contained=transported,
                      final_contained=final_count, required=p["count"], final_bowl_pose=scene.measure()["bowl"]["pose"])
        if transported != p["count"] or final_count != p["count"]:
            raise RuntimeError("Gentle loaded-bowl transport spilled: " + str(result))
    elif task == "L2-05":
        motor = scene.drives["lid_hinge", "rotY"]
        motor.CreateStiffnessAttr(0.1)
        motor.CreateDampingAttr(0.01)
        motor.CreateMaxForceAttr(0.12)
        slider = scene.drives["latch_spring", "transX"]

        def angle():
            rotation = np.linalg.inv(pose_transform(scene.initial["lid"])) @ pose_transform(scene.measure()["lid"]["pose"])
            return float(math.atan2(rotation[2, 0], rotation[0, 0]))

        motor.CreateTargetPositionAttr(110.0)
        advance(scene, 2)
        locked = angle()
        motor.CreateTargetPositionAttr(0.0)
        advance(scene, 1)
        slider.CreateTargetPositionAttr(-0.018)
        advance(scene, 1)
        motor.CreateTargetPositionAttr(110.0)
        advance(scene, 4)
        opened = angle()
        motor.CreateStiffnessAttr(0.0)
        motor.CreateDampingAttr(0.005)
        slider.CreateTargetPositionAttr(0.0)
        advance(scene, 3)
        resting_open = angle()
        slider.CreateTargetPositionAttr(-0.018)
        advance(scene, 1)
        motor.CreateStiffnessAttr(0.1)
        motor.CreateTargetPositionAttr(0.0)
        advance(scene, 4)
        slider.CreateTargetPositionAttr(0.0)
        advance(scene, 2)
        result.update(locked_angle_rad=locked, opened_angle_rad=opened,
                      unsupported_open_angle_rad=resting_open, reclosed_angle_rad=angle())
        if locked > 0.2 or opened < 1.8 or resting_open < 1.75 or abs(angle()) > 0.1:
            raise RuntimeError("Single-arm latch mechanics probe failed: " + str(result))
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
