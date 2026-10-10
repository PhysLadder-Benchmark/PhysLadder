"""Native bimanual bottle completion accepts removal without arbitrary turn quotas."""

import unittest
import numpy as np
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.l1.specs import pose, local_corners
from test_household_tasks import measurement


class BottleTaskTests(unittest.TestCase):
    def setUp(self):
        self.spec = build_spec("L3-03")
        self.rows = measurement(self.spec)
        self.e = BenchEvaluator(
            self.spec, {a.name: a.initial for a in self.spec.design.articles}
        )
        self.cap = self.spec.design.articles[1]

    def place(self, p):
        r = self.rows["cap"]
        r["pose"] = list(p)
        t = pose_transform(p)
        r["corners"] = (local_corners(self.cap) @ t[:3, :3].T + t[:3, 3]).tolist()
        return r

    def test_native_cap_fits_and_bottle_is_free(self):
        _, c = resolve_task("L3-03")
        self.assertLess(
            np.ptp(local_corners(self.cap)[:, 0]), c.robot.gripper.width_limits_m[1]
        )
        self.assertEqual(c.arms, ("left", "right"))
        self.assertEqual(c.observation_arms, ())
        item = self.spec.parameters["native_instances"][0]
        self.assertNotIn("Body", item.get("static", []))
        self.assertAlmostEqual(item["initial_body_transforms"]["Cap"][2], 0.0075)
        self.assertEqual(self.e.update(1, self.rows).status, "running")

    def test_detach_then_released_support_completes_without_wait(self):
        r = self.place(
            pose(self.cap.initial[0], self.cap.initial[1], self.cap.initial[2] + 0.035)
        )
        r["fingers"]["right"] = [1, 1]
        self.rows["bottle"]["fingers"]["left"] = [1, 1]
        self.assertEqual(self.e.update(1, self.rows).status, "running")
        r = self.place(pose(0.10, 1.25, 0.78 - 0.1742))
        r["fingers"]["right"] = [0, 0]
        r["bench_force_n"] = 0.0035 * 9.81
        r["angular_velocity"] = [0, 0, 2.0]
        self.assertEqual(self.e.update(2, self.rows).status, "succeeded")
        r["bench_force_n"] = 0
        self.assertEqual(self.e.update(3, self.rows).status, "running")

    def test_air_contact_or_no_manipulation_do_not_pass(self):
        r = self.place(pose(0.10, 1.25, 0.78 - 0.1742))
        r["bench_force_n"] = 0.0343
        self.assertEqual(self.e.update(1, self.rows).status, "running")
        self.e.touched.add("cap")
        r["pair_forces"]["bottle"] = 0.02
        self.assertEqual(self.e.update(2, self.rows).status, "running")
        r["pair_forces"]["bottle"] = 0
        r["fingers"]["right"] = [1, 1]
        self.assertEqual(self.e.update(3, self.rows).status, "running")

    def test_cap_removal_without_other_arm_support_is_not_bimanual(self):
        r = self.place(
            pose(self.cap.initial[0], self.cap.initial[1], self.cap.initial[2] + 0.035)
        )
        r["fingers"]["right"] = [1, 1]
        self.e.update(1, self.rows)
        self.assertNotIn("cap_detached", self.e.flags)
        self.rows["bottle"]["fingers"]["left"] = [1, 1]
        self.e.update(2, self.rows)
        self.assertIn("cap_detached", self.e.flags)
        r = self.place(pose(0.10, 1.25, 0.78 - 0.1742))
        r["fingers"]["right"] = [0, 0]
        r["bench_force_n"] = 0.0343
        self.rows["bottle"]["fingers"]["left"] = [0, 0]
        self.assertEqual(self.e.update(3, self.rows).status, "succeeded")
        self.rows["bottle"]["bench_force_n"] = 0
        self.assertEqual(self.e.update(4, self.rows).status, "running")
