"""Native L2 dimensions, roles, clearances and scoring boundaries, offline."""

import unittest

import numpy as np
from test_household_tasks import measurement

from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.scoring_l2 import ball_contained
from robot_tasks.tasks.bench.spec import build_spec


class L2LayoutTests(unittest.TestCase):
    def test_roles_and_front_extraction_use_native_lab_frame(self):
        for task in ("L2-03", "L2-07"):
            with self.subTest(task=task):
                entry, cfg = resolve_task(task)
                entry.factory.prepare_config(cfg)
                self.assertEqual(cfg.arms, ("left",))
                self.assertEqual(cfg.observation_arms, ("right",))
                self.assertEqual(cfg.raw["simulation"]["environment"], "tuojing_lab_v4")
                spec = build_spec(task)
                a = spec.design.articles[0]
                outward = np.asarray(spec.parameters["outward_axis_world"])
                np.testing.assert_allclose(outward, [0, 1, 0])
                travel = np.asarray(a.goal[:3]) - a.initial[:3]
                self.assertGreater(travel @ outward, 0.065)
                np.testing.assert_allclose(travel[[0, 2]], 0)

    def test_full_size_book_grips_exposed_thickness_and_clears_neighbours(self):
        s = build_spec("L2-03")
        a, p = s.design.articles[0], s.parameters
        self.assertAlmostEqual(a.mass, 0.3789)
        np.testing.assert_allclose(
            np.subtract(*a.asset_bounds[::-1]), [0.14, 0.22, 0.02]
        )
        self.assertLess(p["grip_thickness_m"], 0.07)
        self.assertGreater(p["grip_thickness_m"], 0.012)
        self.assertGreater(p["exposed_length_m"], 0.06)
        self.assertGreater(
            a.goal[1] - 0.11 - p["stack_front_world"][1], p["clearance_m"]
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
            row["pose"][0] += 0.15
            row["corners"] = (np.asarray(row["corners"]) + [0.15, 0, 0]).tolist()
            row["fingers"]["left"] = [1, 1]
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
        r["pose"][1] += 0.04
        r["corners"] = (np.asarray(r["corners"]) + [0, 0.04, 0]).tolist()
        r["fingers"]["left"] = [1, 1]
        e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
        for i in range(1, 15):
            result = e.update(i * 0.1, rows)
            self.assertEqual(result.status, "running")
        self.assertLess(result.metrics["criteria"]["rear_clearance_m"], 0)

    def test_native_pearl_packing_is_contained_nonintersecting_and_declared(self):
        s = build_spec("L2-04")
        p, balls = s.parameters, s.design.articles[1:]
        self.assertEqual(len(balls), p["count"])
        self.assertGreater(p["count"], 600)
        self.assertFalse(s.joints)
        self.assertEqual(p["pearl_collision"], "outer_sphere")
        pts = np.asarray([a.initial[:3] for a in balls])
        for i, a in enumerate(balls):
            self.assertEqual(a.asset, "tuojing:pearl_8mm")
            self.assertTrue(
                ball_contained(pts[i] - s.design.articles[0].initial[:3], p)
            )
            if i:
                self.assertTrue(
                    np.all(
                        np.linalg.norm(pts[:i] - pts[i], axis=1)
                        >= 2 * p["ball_radius_m"]
                    )
                )
        self.assertLess(p["ball_friction"], 0.1)
        self.assertAlmostEqual(s.design.articles[0].mass, 0.12 + 0.016)

    def test_ball_on_rim_below_floor_or_beyond_native_wall_is_not_contained(self):
        p = build_spec("L2-04").parameters
        self.assertTrue(ball_contained((0, 0, 0.02), p))
        for pt in ((0, 0, 0.055), (0.068, 0, 0.05), (0.05, 0, 0.02), (0, 0, -0.01)):
            self.assertFalse(ball_contained(pt, p), pt)

    def test_drawer_score_fits_native_90mm_rail_and_adapter_fits_r5(self):
        s = build_spec("L2-07")
        p = s.parameters
        self.assertLess(p["required_open_m"], p["max_travel_m"])
        self.assertAlmostEqual(p["max_travel_m"], 0.09)
        item = p["native_instances"][0]
        self.assertEqual(item["tracked"], {"drawer": "Drawer1"})
        self.assertEqual(item["static"], ["Body", "Drawer0"])
        # Native rail moves along local -Y, transformed into world +Y.
        np.testing.assert_allclose(
            -pose_transform(item["pose"])[:3, 1], p["outward_axis_world"], atol=1e-12
        )
        grip = item["adapters"]["Drawer1"][-1]
        self.assertGreater(grip.size[0], 0.012)
        self.assertLess(grip.size[0], 0.070)
        self.assertLess(grip.position[1] + grip.size[1] / 2, -0.0725)
        self.assertAlmostEqual(s.design.articles[0].mass, 0.08 + 0.008)


if __name__ == "__main__":
    unittest.main()
