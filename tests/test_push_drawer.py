"""Latch hysteresis and task-scoring regressions; no model or simulator calls."""

import unittest

import numpy as np

from test_household_tasks import measurement
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.push_drawer import PushLatch, build


class PushDrawerTests(unittest.TestCase):
    def setUp(self):
        self.spec = build()
        self.rows = measurement(self.spec)
        self.initial = {a.name: a.initial for a in self.spec.design.articles}
        self.evaluator = BenchEvaluator(self.spec, self.initial)
        self.time = 0.0

    def assess(self, travel, phase, fingers=(0, 0), dt=0.1):
        self.time += dt
        upper = self.rows["upper_drawer"]
        upper["pose"][:3] = (np.asarray(self.initial["upper_drawer"][:3])
                             + travel * np.asarray(self.spec.parameters["outward_axis_world"])).tolist()
        upper["push_latch"] = {"phase": phase}
        upper["fingers"]["left"] = list(fingers)
        return self.evaluator.update(self.time, self.rows)

    def test_incomplete_or_brief_press_and_outward_pull_do_not_unlock(self):
        latch = PushLatch()
        for travel, dt in ((0.001, 1), (-0.004, 1), (-0.010, .02), (0.001, 1)):
            result = latch.update(travel, dt)
            self.assertEqual(result["phase"], "locked")
            self.assertLess(result["upper_limit_m"], 0.002)

    def test_full_press_needs_rebound_and_spring_cannot_complete_pull(self):
        latch = PushLatch()
        self.assertEqual(latch.update(-.010, .1)["phase"], "armed")
        self.assertEqual(latch.update(-.010, 1)["phase"], "armed")
        self.assertEqual(latch.update(-.002, .1)["phase"], "open")
        result = latch.update(.10, .1)
        self.assertEqual(result["stiffness"], 0.0)
        self.assertEqual(PushLatch().update(0, .1)["phase"], "locked")

    def test_target_is_upper_and_contact_task_does_not_enable_lift_guard(self):
        spec, config = resolve_task("L3-16")
        self.assertEqual(spec.factory.prepare_config(config).grasp_check_mode, "contact_inspection")
        self.assertEqual(config.arms, ("left",))
        self.assertEqual(config.observation_arms, ("right",))
        self.assertGreater(self.initial["upper_drawer"][2], self.initial["lower_drawer"][2])
        self.assertEqual(self.spec.joints[1].axes, {})

    def test_front_and_full_pull_stroke_face_manipulation_arm(self):
        _, config = resolve_task("L3-16")
        base = next(m.base_pose_wxyz[:3] for m in config.robot.mounts if m.name == "left")
        upper = self.spec.design.articles[0]
        initial = pose_transform(upper.initial)
        front = next(p for p in upper.parts if p.name == "upper_front")
        tab = next(p for p in upper.parts if p.name == "upper_tab")
        front_world = initial[:3, :3] @ front.position + initial[:3, 3]
        normal = initial[:3, 0]
        self.assertGreater(float(normal @ (np.asarray(base) - front_world)), 0)
        np.testing.assert_allclose(normal, [-1, 0, 0], atol=1e-9)
        np.testing.assert_allclose(normal, self.spec.parameters["outward_axis_world"], atol=1e-9)
        self.assertLess(upper.goal[0], upper.initial[0])
        full_pull_tab = pose_transform(upper.goal) @ (*tab.position, 1)
        self.assertGreater(full_pull_tab[0], base[0] + .20)
        back = next(p for p in self.spec.design.fixtures if p.name == "cabinet_back")
        self.assertGreater(back.position[0], initial[0, 3])

    def test_world_x_increase_is_pressing_not_opening(self):
        upper = self.rows["upper_drawer"]
        upper["pose"][0] += .010
        upper["fingers"]["left"] = [1, 0]
        upper["push_latch"] = {"phase": "armed"}
        result = self.evaluator.update(.1, self.rows)
        self.assertEqual(result.status, "running")
        self.assertAlmostEqual(result.metrics["criteria"]["travel_m"], -.010)
        self.assertTrue(result.metrics["criteria"]["pressed_by_manipulation_arm"])
        self.assertFalse(result.metrics["criteria"]["grasped_pull_observed"])

    def test_final_position_and_latch_state_without_process_are_not_success(self):
        for _ in range(20):
            self.assertEqual(self.assess(.11, "open").status, "running")

    def test_press_requires_actual_manipulation_contact(self):
        self.assess(-.01, "armed")
        self.assess(.03, "open")
        self.assess(.11, "open", (1, 1))
        for _ in range(20):
            self.assertEqual(self.assess(.11, "open").status, "running")

    def test_holding_while_pulling_out_is_not_released_spring_popout(self):
        self.assess(-.01, "armed", (1, 0))
        self.assess(.03, "open", (1, 1))
        self.assess(.11, "open", (1, 1))
        for _ in range(20):
            self.assertEqual(self.assess(.11, "open").status, "running")

    def test_valid_opening_needs_grasped_pull_release_and_stable_hold(self):
        self.assess(-.01, "armed", (1, 0))
        self.assess(.03, "open")
        for _ in range(12):
            self.assertEqual(self.assess(.03, "open").status, "running")
        self.assertEqual(self.assess(.105, "open", (1, 1)).status, "running")
        self.assertEqual(self.assess(.105, "open").status, "running")
        result = self.assess(.105, "open", dt=1.05)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["criteria"]["target_layer"], "upper")
        fresh = BenchEvaluator(self.spec, self.initial)
        self.assertEqual(fresh.update(.1, self.rows).status, "running")

    def test_lower_layer_operation_fails(self):
        self.rows["lower_drawer"]["fingers"]["left"] = [1, 1]
        self.assertEqual(self.assess(0, "locked").status, "running")
        result = self.assess(0, "locked", dt=.2)
        self.assertEqual(result.status, "failed")
        self.assertIn("lower", result.reason)

    def test_excessive_force_speed_and_broken_guide_fail(self):
        for field, value in (("linear_velocity", [0.5, 0, 0]), ("pose", [0.335, .30, .101, 1, 0, 0, 0])):
            with self.subTest(field=field):
                rows = measurement(self.spec)
                rows["upper_drawer"][field] = value
                result = BenchEvaluator(self.spec, self.initial).update(.1, rows)
                self.assertEqual(result.status, "failed")
        self.assertEqual(self.assess(-.01, "armed", (45, 0)).status, "failed")


if __name__ == "__main__":
    unittest.main()
