"""Direct-push process and concave target scoring; no model or simulator calls."""

import math
import unittest

import numpy as np

from test_household_tasks import measurement
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.push_t import build, footprint_outside_area
from robot_tasks.tasks.l1.specs import local_corners, pose


class PushTTests(unittest.TestCase):
    def setUp(self):
        self.spec = build()
        self.article = self.spec.design.articles[0]
        self.rows = measurement(self.spec)
        self.block = self.rows["t_block"]
        self.evaluator = BenchEvaluator(self.spec, {self.article.name: self.article.initial})
        self.time = 0.0

    def place(self, xyz, yaw=0.0):
        self.block["pose"] = list(pose(*xyz, yaw=yaw))
        t = pose_transform(self.block["pose"])
        self.block["corners"] = (local_corners(self.article) @ t[:3, :3].T + t[:3, 3]).tolist()

    def assess(self, dt=.1):
        self.time += dt
        return self.evaluator.update(self.time, self.rows)

    def push_witness(self):
        self.block["fingers"]["left"] = [1, 0]
        self.assess()
        self.place((.39, .425, .013), yaw=.65)
        result = self.assess()
        self.assertTrue(result.metrics["criteria"]["gripper_push_witness"])

    def outside(self, dx=0, dy=0, yaw=0):
        p = pose(self.article.goal[0] + dx, self.article.goal[1] + dy,
                 self.article.goal[2], yaw=yaw)
        return footprint_outside_area(self.article, p, self.spec.parameters["target_rectangles"])

    def test_only_block_and_noncolliding_outline_no_lift_gate(self):
        self.assertEqual([a.name for a in self.spec.design.articles], ["t_block"])
        self.assertEqual(len(self.spec.design.fixtures), 8)
        self.assertTrue(all(not p.collision and p.size[1] <= .002 for p in self.spec.design.fixtures))
        binding, config = resolve_task("L3-14")
        prepared = binding.factory.prepare_config(config)
        self.assertEqual(prepared.grasp_check_mode, "contact_inspection")
        self.assertEqual(prepared.observation_arms, ("right",))
        self.assertEqual(config.source.name, "l3_14_gripper_push_t.toml")

    def test_target_really_accepts_relaxed_position_and_yaw(self):
        self.assertLess(self.outside(dx=.018), 1e-9)
        self.assertLess(self.outside(yaw=math.radians(20)), 1e-9)
        self.assertGreater(self.outside(dx=.025), 1e-6)
        self.assertGreater(self.outside(dx=.12), .003)

    def test_notch_is_excluded_even_inside_overall_target_bounding_box(self):
        self.place((.408, .22, .013), yaw=math.radians(20))
        xy = np.asarray(self.block["corners"])[:, :2] - self.article.goal[:2]
        self.assertTrue(np.all((xy[:, 0] >= -.07) & (xy[:, 0] <= .07)))
        self.assertTrue(np.all((xy[:, 1] >= -.08) & (xy[:, 1] <= .06)))
        self.assertGreater(self.outside(dx=.018, yaw=math.radians(20)), 1e-6)

    def test_tilted_box_height_is_part_of_projected_footprint(self):
        p = pose(self.article.goal[0] + .0195, self.article.goal[1],
                 self.article.goal[2], pitch=.1)
        self.assertGreater(footprint_outside_area(
            self.article, p, self.spec.parameters["target_rectangles"]), 1e-6)

    def test_position_or_stationary_touch_alone_cannot_complete(self):
        self.place(self.article.goal[:3])
        self.block["fingers"]["left"] = [1, 0]
        self.assess()
        self.block["fingers"]["left"] = [0, 0]
        for _ in range(15):
            result = self.assess()
            self.assertEqual(result.status, "running")
        self.assertFalse(result.metrics["criteria"]["gripper_push_witness"])

    def test_direct_push_then_withdraw_and_stabilize_completes(self):
        self.push_witness()
        # 18 mm centre offset exceeds the retired 12 mm rule but fits the outline.
        self.place((.408, .22, .013))
        self.assertEqual(self.assess(1.1).status, "running")
        self.block["fingers"]["left"] = [0, 0]
        self.assertEqual(self.assess().status, "running")
        self.assertEqual(self.assess(1.05).status, "succeeded")
        fresh = BenchEvaluator(self.spec, {self.article.name: self.article.initial})
        self.assertEqual(fresh.update(.1, self.rows).status, "running")

    def test_sideways_T_is_not_success_even_if_it_fits(self):
        self.push_witness()
        self.place((.395, .225, .013), yaw=math.pi / 2)
        self.block["fingers"]["left"] = [0, 0]
        result = self.assess(1.1)
        self.assertTrue(result.metrics["criteria"]["inside_target_outline"])
        self.assertEqual(result.status, "running")

    def test_protruding_edge_or_motion_cannot_complete(self):
        self.push_witness()
        self.block["fingers"]["left"] = [0, 0]
        self.place((.408, .22, .013), yaw=math.radians(20))
        self.assertEqual(self.assess(1.1).status, "running")
        self.place(self.article.goal[:3])
        self.block["linear_velocity"] = [.04, 0, 0]
        self.assertEqual(self.assess(1.1).status, "running")

    def test_incidental_lift_and_observer_touch_allow_recovery(self):
        self.place((.39, .30, .04))
        self.assertEqual(self.assess().status, "running")
        self.block["fingers"]["right"] = [1, 0]
        self.assertEqual(self.assess().status, "running")
        self.block["fingers"]["right"] = [0, 0]
        self.place(self.article.initial[:3])
        self.push_witness()
        self.place(self.article.goal[:3])
        self.block["fingers"]["left"] = [0, 0]
        self.assess()
        result = self.assess(1.1)
        self.assertEqual(result.status, "succeeded")
        self.assertTrue(result.metrics["quality_warnings"])

    def test_grasped_carrying_is_still_a_process_violation(self):
        self.place((.39, .30, .04))
        self.block["bench_force_n"] = 0
        self.block["fingers"]["left"] = [1, 1]
        self.assertEqual(self.assess().status, "running")
        result = self.assess(.3)
        self.assertEqual(result.status, "failed")
        self.assertIn("carried", result.reason)

    def test_observer_manipulation_is_still_a_process_violation(self):
        self.block["fingers"]["right"] = [1, 0]
        self.assess()
        self.place((self.article.initial[0] + .01, *self.article.initial[1:3]))
        result = self.assess(.2)
        self.assertEqual(result.status, "failed")
        self.assertIn("observation arm manipulated", result.reason)


if __name__ == "__main__":
    unittest.main()
