"""Native nut goal and workflow tests; no simulator or model requests."""

import math
import unittest
from dataclasses import replace

import numpy as np

from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform, transform_pose
from robot_tasks.tasks.l1.specs import pose
from robot_tasks.tasks.screw.nut import NutEvaluator, NutTask, nut_spec


def sample(turn=0.0, rise=0.0, lifted=True, hands=True, body_pose=None, contact=0.02):
    spec = nut_spec()
    body = (
        pose(0.19565, 1.30, 0.84 if lifted else 0.781)
        if body_pose is None
        else body_pose
    )
    nut = transform_pose(
        pose_transform(body) @ pose_transform(pose(0, 0, 0.0875 + rise, yaw=turn))
    )
    rows = {}
    for article, current, arm in zip(
        spec.design.articles, (body, nut), ("left", "right")
    ):
        t = pose_transform(current)
        lo, hi = article.asset_bounds
        corners = np.array(
            [
                (x, y, z)
                for x in (lo[0], hi[0])
                for y in (lo[1], hi[1])
                for z in (lo[2], hi[2])
            ]
        )
        rows[article.name] = {
            "pose": current,
            "linear_velocity": [0, 0, 0],
            "angular_velocity": [0, 0, 0],
            "bench_force_n": 0 if lifted else 0.5,
            "fingers": {
                a: [0.2, 0.2] if hands and a == arm else [0.0, 0.0]
                for a in ("left", "right")
            },
            "pair_forces": {"nut" if arm == "left" else "anchor": contact},
            "corners": (corners @ t[:3, :3].T + t[:3, 3]).tolist(),
        }
    return rows


class NutTaskTests(unittest.TestCase):
    def test_shared_workflow_two_hands_and_local_physics_override(self):
        entry, cfg = resolve_task("L3-05")
        self.assertIs(entry.factory, NutTask)
        cfg = entry.factory.prepare_config(cfg)
        self.assertEqual(cfg.arms, ("left", "right"))
        self.assertEqual(cfg.observation_arms, ())
        self.assertAlmostEqual(cfg.physics_dt_s, 1 / 1920)
        self.assertEqual(cfg.raw["execution"]["max_decisions"], 100)
        self.assertEqual(cfg.robot.gripper.width_limits_m, (0.010, 0.070))
        self.assertEqual(cfg.grasp_check_mode, "contact_inspection")
        _, other = resolve_task("L3-03")
        self.assertAlmostEqual(other.physics_dt_s, 1 / 240)
        self.assertEqual(other.robot.gripper.width_limits_m, (0.012, 0.070))
        with self.assertRaises(ValueError):
            NutTask.prepare_config(replace(cfg, physics_dt_s=1 / 240))
        with self.assertRaises(ValueError):
            NutTask.prepare_config(
                replace(cfg, arms=("left",), observation_arms=("right",))
            )

    def test_preloosened_but_free_and_still_engaged(self):
        s = nut_spec()
        item = s.parameters["native_instances"][0]
        self.assertNotIn("static", item)
        self.assertFalse(s.joints)
        self.assertAlmostEqual(item["initial_body_transforms"]["Nut"][2], 0.0125)
        self.assertAlmostEqual(
            (s.parameters["shaft_tip_z"] - s.parameters["initial_nut_bottom_z"])
            / s.parameters["thread_pitch_m"],
            2.0,
        )
        self.assertEqual(
            NutEvaluator(s).update(0, sample(lifted=False, hands=False)).status,
            "running",
        )

    def test_detachment_finishes_immediately_while_held(self):
        e = NutEvaluator(nut_spec())
        e.update(0, sample())
        result = e.update(0.1, sample(0.5, 0.0001))
        self.assertEqual(result.status, "running")
        result = e.update(0.2, sample(1.0, 0.003, contact=0))
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["criteria"]["timed_hold_s"], 0)

    def test_no_spin_or_no_bimanual_grip_cannot_earn_success(self):
        for hands, angle in ((False, 1.0), (True, 0.0)):
            with self.subTest(hands=hands, angle=angle):
                e = NutEvaluator(nut_spec())
                e.update(0, sample(hands=hands))
                r = e.update(0.1, sample(angle, 0.003, hands=hands, contact=0))
                self.assertEqual(r.status, "running")

    def test_rigid_assembly_motion_does_not_count_as_unscrewing(self):
        e = NutEvaluator(nut_spec())
        e.update(0, sample())
        r = e.update(
            0.1, sample(body_pose=pose(0.2, 1.3, 0.9, roll=0.5, pitch=0.3, yaw=0.7))
        )
        self.assertAlmostEqual(r.metrics["criteria"]["relative_turn_rad"], 0)
        self.assertAlmostEqual(r.metrics["criteria"]["relative_rise_m"], 0)
        self.assertEqual(r.status, "running")

    def test_tilted_body_uses_relative_thread_clearance(self):
        e = NutEvaluator(nut_spec())
        body = pose(0.2, 1.3, 0.95, roll=0.6, pitch=0.4, yaw=0.7)
        e.update(0, sample(body_pose=body))
        r = e.update(0.1, sample(0.5, 0.003, body_pose=body, contact=0))
        self.assertEqual(r.status, "succeeded")
        self.assertAlmostEqual(r.metrics["criteria"]["tip_clearance_m"], 0.0005)

    def test_scoring_cache_allows_paused_polling(self):
        from types import SimpleNamespace
        from unittest.mock import Mock

        task = NutTask()
        task.scene = SimpleNamespace(
            spec=nut_spec(), reset=Mock(), measure=Mock(return_value=sample())
        )
        task.reset(None, 0)
        state = SimpleNamespace(values={"simulation_time_s": 1.0})
        first = task.evaluate(state, None)
        self.assertIs(first, task.evaluate(state, None))
        task.scene.measure.assert_called_once()
