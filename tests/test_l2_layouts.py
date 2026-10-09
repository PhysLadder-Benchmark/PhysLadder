"""Native L2 dimensions, roles, clearances and scoring boundaries, offline."""

import unittest

import numpy as np
from test_household_tasks import measurement

from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.spec import build_spec


class L2LayoutTests(unittest.TestCase):
    def test_roles_and_extraction_directions_use_native_lab_frame(self):
        for task in ("L2-03", "L2-07"):
            with self.subTest(task=task):
                entry, cfg = resolve_task(task)
                entry.factory.prepare_config(cfg)
                self.assertEqual(cfg.arms, ("right",))
                self.assertEqual(cfg.observation_arms, ("left",))
                self.assertEqual(cfg.raw["simulation"]["environment"], "tuojing_lab_v4")
                spec = build_spec(task)
                a = spec.design.articles[0]
                outward = np.asarray(spec.parameters["outward_axis_world"])
                np.testing.assert_allclose(outward, [1, 0, 0] if task == "L2-03" else [0, 1, 0])
                travel = np.asarray(a.goal[:3]) - a.initial[:3]
                self.assertGreater(travel @ outward, 0.04 if task == "L2-07" else 0.065)
                np.testing.assert_allclose(travel - (travel @ outward) * outward, 0, atol=1e-12)

    def test_full_size_book_grips_exposed_thickness_and_clears_neighbours(self):
        s = build_spec("L2-03")
        a, p = s.design.articles[0], s.parameters
        self.assertAlmostEqual(a.mass, 0.3789)
        np.testing.assert_allclose(
            np.subtract(*a.asset_bounds[::-1]), [0.14, 0.22, 0.02]
        )
        self.assertLess(p["grip_thickness_m"], 0.07)
        self.assertGreater(p["grip_thickness_m"], 0.012)
        self.assertAlmostEqual(p["exposed_length_m"], 0.04)
        self.assertGreater(
            a.goal[0] - 0.07 - p["stack_front_world"][0], p["clearance_m"]
        )
        lower, upper = p["native_instances"][:2]
        self.assertEqual(lower["static"], [""])
        self.assertEqual(upper["static"], [""])
        self.assertLess(lower["pose"][2] + 0.01, a.initial[2] - 0.01)
        self.assertGreater(upper["pose"][2] - 0.01, a.initial[2] + 0.01)
        # This is extraction, not a sub-millimetre wedging task. Preserve room
        # for the measured controller deflection while retaining lower friction.
        self.assertGreaterEqual(upper["pose"][2] - a.initial[2] - 0.02, 0.004)

    def test_sideways_motion_cannot_count_as_extraction(self):
        for task, name in (("L2-03", "middle_book"), ("L2-07", "drawer")):
            s = build_spec(task)
            rows = measurement(s)
            row = rows[name]
            transverse = np.array([0, .15, 0] if task == "L2-03" else [.15, 0, 0])
            row["pose"][:3] = (np.asarray(row["pose"][:3]) + transverse).tolist()
            row["corners"] = (np.asarray(row["corners"]) + transverse).tolist()
            row["fingers"][s.arms[0]] = [1, 1]
            e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
            for i in range(1, 15):
                result = e.update(i * 0.1, rows)
            self.assertEqual(result.status, "invalid" if task == "L2-07" else "running")
            self.assertNotIn("middle_extracted_while_held", e.flags)
            self.assertNotIn("opened_while_held", e.flags)

    def test_partly_extracted_book_is_not_complete(self):
        s = build_spec("L2-03")
        rows = measurement(s)
        r = rows["middle_book"]
        r["pose"][0] += 0.04
        r["corners"] = (np.asarray(r["corners"]) + [0.04, 0, 0]).tolist()
        r["fingers"][s.arms[0]] = [1, 1]
        e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
        for i in range(1, 15):
            result = e.update(i * 0.1, rows)
            self.assertEqual(result.status, "running")
        self.assertLess(result.metrics["criteria"]["rear_clearance_m"], 0)

    def test_grasped_book_extraction_finishes_without_stationary_hold(self):
        s = build_spec("L2-03")
        rows = measurement(s)
        row = rows["middle_book"]
        row["pose"][0] += .11
        row["corners"] = (np.asarray(row["corners"]) + [.11, 0, 0]).tolist()
        row["fingers"][s.arms[0]] = [1, 1]
        row["linear_velocity"] = [.1, 0, 0]
        e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
        result = e.update(.1, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["stable_duration_s"], 0)
        self.assertTrue(result.metrics["criteria"]["extracted_while_held"])

    def test_extracted_book_on_table_does_not_need_regrasp_but_needs_pull_history(self):
        for history in (False, True):
            with self.subTest(history=history):
                s = build_spec("L2-03")
                rows = measurement(s)
                row = rows["middle_book"]
                dz = s.parameters["table_z"] - np.min(np.asarray(row["corners"])[:, 2])
                row["pose"][:3] = (np.asarray(row["pose"][:3]) + [.15, 0, dz]).tolist()
                row["corners"] = (np.asarray(row["corners"]) + [.15, 0, dz]).tolist()
                row["bench_force_n"] = 3.717
                e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
                if history:
                    e.flags.add("middle_extracted_while_held")
                result = e.update(.1, rows)
                self.assertEqual(result.status, "succeeded" if history else "running")
                self.assertFalse(result.metrics["criteria"]["held_by_manipulation_arm"])
                self.assertTrue(result.metrics["criteria"]["table_supported"])
                # Past extraction does not qualify an unsupported falling book.
                row["bench_force_n"] = 0
                self.assertEqual(e.update(.2, rows).status, "running")
                row["corners"] = (np.asarray(row["corners"]) - [0, 0, .1]).tolist()
                row["pose"][2] -= .1
                self.assertEqual(e.update(.3, rows).status, "failed")


    def test_drawer_keeps_native_handle_mass_bounds_and_90mm_rail(self):
        s = build_spec("L2-07")
        p = s.parameters
        self.assertLess(p["required_open_m"], p["max_travel_m"])
        self.assertAlmostEqual(p["max_travel_m"], 0.09)
        item = p["native_instances"][0]
        self.assertEqual(item["tracked"], {"drawer": "Drawer1"})
        self.assertEqual(item["static"], ["Body", "Drawer0"])
        self.assertAlmostEqual(p["initial_open_m"], .03)
        np.testing.assert_allclose(item["initial_body_offsets"]["Drawer1"], [0, -.03, 0])
        np.testing.assert_allclose(np.subtract(s.design.articles[0].initial[:3],
                                             p["closed_pose"][:3]), [0, .03, 0])
        # Native rail moves along local -Y, transformed into world +Y.
        np.testing.assert_allclose(
            -pose_transform(item["pose"])[:3, 1], p["outward_axis_world"], atol=1e-12
        )
        self.assertNotIn("adapters", item)
        self.assertNotIn("mass_additions", item)
        self.assertFalse(p.get("grasp_adapters"))
        self.assertAlmostEqual(s.design.articles[0].mass, 0.080)
        self.assertAlmostEqual(s.design.articles[0].asset_bounds[0][1], -0.0735)


if __name__ == "__main__":
    unittest.main()
