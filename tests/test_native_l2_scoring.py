"""Regressions for native masses, axial spin and soft FEM scoring."""

import unittest

import numpy as np
from test_household_tasks import measurement

from robot_tasks.robots.transforms import (
    origin_transform,
    pose_transform,
    transform_pose,
)
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.orange_evaluation import (
    OrangeEvaluator,
    deformation_metrics,
    motion_metrics,
)
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.types import InvalidSimulationState


def soft_row(spec, *, at_goal=False, lifted=False, strain=0.0, touching=False):
    a = spec.design.articles[0]
    p = np.asarray((a.goal if at_goal else a.initial)[:3]).copy()
    z = spec.parameters["support_z"] + (0.06 if lifted else 0.0)
    return {
        "orange": {
            "bounds_min": [p[0] - 0.025, p[1] - 0.013, z],
            "bounds_max": [p[0] + 0.025, p[1] + 0.013, z + 0.017],
            "centroid": [p[0], p[1], z + 0.0085],
            "collision_bounds_min": [p[0] - 0.025, p[1] - 0.013, z],
            "collision_bounds_max": [p[0] + 0.025, p[1] + 0.013, z + 0.017],
            "finger_distances_m": [0.001, 0.001, 0.10, 0.10] if touching else [0.1] * 4,
            "speed_m_s": 0.0,
            "centroid_speed_m_s": 0.0,
            "shape_speed_p95_m_s": 0.0,
            "centroid": [p[0], p[1], z + 0.0085],
            "strain_p95": strain,
            "strain_max": strain,
            "minimum_volume_ratio": 1.0,
        }
    }


class NativeL2ScoringTests(unittest.TestCase):



    def test_drawer_release_near_threshold_can_finish_after_passive_travel(self):
        spec = build_spec("L2-07")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        row = rows["drawer"]
        row["fingers"]["right"] = [1, 1]
        e.update(.1, rows)
        row["pose"][1] += .074 - spec.parameters["initial_open_m"]
        e.update(.2, rows)
        row["fingers"]["right"] = [0, 0]
        row["pose"][1] += .006
        e.update(.3, rows)
        self.assertEqual(e.update(.9, rows).status, "succeeded")

    def test_drawer_touch_or_small_oscillations_do_not_fake_a_grasped_pull(self):
        spec = build_spec("L2-07")
        for oscillate in (False, True):
            rows = measurement(spec)
            row = rows["drawer"]
            row["pose"][1] += .08 - spec.parameters["initial_open_m"]
            start = row["pose"][1]
            e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
            row["fingers"]["right"] = [1, 1]
            for i in range(1, 15):
                row["pose"][1] = start + (.001 * (i % 2) if oscillate else 0)
                e.update(i * .1, rows)
            row["fingers"]["right"] = [0, 0]
            e.update(1.5, rows)
            self.assertEqual(e.update(2.2, rows).status, "running")

    def test_drawer_preopened_layout_scores_absolute_opening_and_native_stop(self):
        spec = build_spec("L2-07")
        rows = measurement(spec)
        row = rows["drawer"]
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        initial = e.update(.1, rows)
        self.assertAlmostEqual(initial.metrics["criteria"]["opening_m"], .03)
        self.assertEqual(initial.status, "running")
        # Only another 50 mm is needed for 80 mm total opening.
        row["fingers"]["right"] = [1, 1]
        e.update(.2, rows)
        row["pose"][1] += .05
        e.update(.3, rows)
        row["fingers"]["right"] = [0, 0]
        e.update(.4, rows)
        result = e.update(1., rows)
        self.assertEqual(result.status, "succeeded")
        self.assertAlmostEqual(result.metrics["criteria"]["opening_m"], .08)
        self.assertAlmostEqual(result.metrics["criteria"]["travel_since_start_m"], .05)
        # Closing to the original zero is legal; preopening cannot shift limits.
        row["pose"] = list(spec.parameters["closed_pose"])
        self.assertEqual(e.update(1.1, rows).status, "running")
        self.assertEqual(e.update(1.5, rows).status, "running")
        row["pose"][1] += .10
        e.update(1.6, rows)
        self.assertEqual(e.update(2., rows).status, "invalid")


    def test_initial_resting_objects_do_not_count_as_completed_placement_stages(self):
        spec = build_spec("L2-01")
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        criteria = evaluator.update(0.1, measurement(spec)).metrics["criteria"]
        self.assertTrue(all(criteria["placed"].values()))
        self.assertFalse(criteria["stages"]["barrel_placed"])
        self.assertFalse(criteria["stages"]["cap_placed"])
        spec = build_spec("L2-06")
        result = OrangeEvaluator(spec).update(0.1, soft_row(spec))
        self.assertFalse(result.metrics["stages"]["lifted_50mm"])

    def test_tilted_native_cap_root_does_not_erase_valid_capped_pickup(self):
        spec = build_spec("L2-01")
        rows = measurement(spec)
        for row in rows.values():
            row["pose"][2] += 0.08
            row["corners"] = (np.asarray(row["corners"]) + [0, 0, 0.08]).tolist()
            row["bench_force_n"] = 0.0
        rows["body"]["fingers"]["left"] = [1, 1]
        body = pose_transform(rows["body"]["pose"])
        pivot = origin_transform((0, 0, spec.parameters["body_tip_z"]), (0, 0, 0))
        cap = (
            body
            @ pivot
            @ origin_transform((0, 0, 0), (0, 0.08, 0))
            @ np.linalg.inv(pivot)
        )
        self.assertGreater(np.linalg.norm(cap[:3, 3] - body[:3, 3]), 0.006)
        rows["cap"]["pose"] = list(transform_pose(cap))
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        result = evaluator.update(0.1, rows)
        self.assertTrue(result.metrics["criteria"]["assembly_lifted"])
        self.assertEqual(result.status, "running")  # Pickup alone is not uncapping.
        # A cap alongside the nib must not count as an assembled marker.
        cap[:3, 3] += body[:3, 0] * 0.03
        rows["cap"]["pose"] = list(transform_pose(cap))
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        self.assertFalse(
            evaluator.update(0.1, rows).metrics["criteria"]["assembly_lifted"]
        )

    def test_cap_mass_based_support_and_own_axis_spin_can_finish(self):
        s = build_spec("L2-01")
        rows = measurement(s)
        e = BenchEvaluator(s, {a.name: a.initial for a in s.design.articles})
        self.assertEqual(e.update(0.1, rows).status, "running")
        for r in rows.values():
            r["pose"][2] += 0.08
            r["corners"] = (np.asarray(r["corners"]) + [0, 0, 0.08]).tolist()
            r["bench_force_n"] = 0.0
        rows["body"]["fingers"]["left"] = [1, 1]
        e.update(0.2, rows)
        rows["cap"]["pose"][0] += 0.05
        rows["cap"]["fingers"]["right"] = [1, 1]
        e.update(0.3, rows)
        self.assertIn("separated_while_held", e.flags)
        for name, r in rows.items():
            r["pose"][2] -= 0.08
            r["corners"] = (np.asarray(r["corners"]) - [0, 0, 0.08]).tolist()
            r["fingers"] = {"left": [0, 0], "right": [0, 0]}
            r["bench_force_n"] = s.parameters[name + "_mass_kg"] * 9.81
            r["angular_velocity"] = [
                9.0,
                0.0,
                0.0,
            ]  # Round native parts lie along world X.
        e.update(0.4, rows)
        result = e.update(1.0, rows)
        self.assertEqual(result.status, "succeeded")
        # Cap's 0.049 N support used to fail a hardcoded 0.05 N threshold.
        self.assertLess(rows["cap"]["bench_force_n"], 0.05)
        rows["cap"]["bench_force_n"] = 0.0
        self.assertEqual(e.update(1.1, rows).status, "running")

    def test_element_strain_ignores_rigid_motion_but_detects_compression(self):
        rest = np.array(
            [[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]], dtype=float
        )
        tet = np.array([[0, 1, 2, 3]])
        inverse = np.array([np.eye(3) * 100])
        t = origin_transform((0.4, 1.2, 0.8), (0.4, -0.2, 0.9))
        transformed = rest @ t[:3, :3].T + t[:3, 3]
        self.assertLess(
            deformation_metrics(inverse, transformed, tet)["strain_p95"], 1e-12
        )
        compressed = rest * [1, 1, 0.6]
        m = deformation_metrics(inverse, compressed, tet)
        self.assertAlmostEqual(m["strain_p95"], 0.4)
        self.assertAlmostEqual(m["minimum_volume_ratio"], 0.6)









if __name__ == "__main__":
    unittest.main()
