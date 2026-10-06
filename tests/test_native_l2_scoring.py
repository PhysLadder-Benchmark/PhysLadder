"""Regressions for native masses, axial spin, inertia and soft FEM scoring."""

import unittest
from types import SimpleNamespace

import numpy as np
from test_household_tasks import measurement

from robot_tasks.robots.transforms import (
    origin_transform,
    pose_transform,
    transform_pose,
)
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.native_l2 import composite_inertia
from robot_tasks.tasks.bench.orange_evaluation import (
    OrangeEvaluator,
    deformation_metrics,
    motion_metrics,
)
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.l1.specs import box
from robot_tasks.types import InvalidSimulationState


def soft_row(spec, *, at_goal=False, lifted=False, strain=0.0, touching=False):
    a = spec.design.articles[0]
    p = np.asarray((a.goal if at_goal else a.initial)[:3]).copy()
    z = spec.parameters["table_z"] + (0.04 if lifted else 0.0)
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
    def test_soft_local_jitter_can_settle_but_bulk_motion_cannot(self):
        spec = build_spec("L2-06")
        previous = np.zeros((100, 3))
        for kind in ("local", "translation", "shape"):
            with self.subTest(kind=kind):
                current = previous.copy()
                if kind == "local":
                    current[0, 0] = .005
                elif kind == "translation":
                    current[:, 0] = .005
                else:
                    current[:50, 0], current[50:, 0] = .005, -.005
                metrics = motion_metrics(current, previous, .1)
                e = OrangeEvaluator(spec)
                e.update(.1, soft_row(spec, lifted=True, touching=True))
                rows = soft_row(spec, at_goal=True)
                rows["orange"].update(metrics)
                e.update(.2, rows)
                result = e.update(.8, rows)
                self.assertEqual(result.status, "succeeded" if kind == "local" else "running")
                self.assertGreater(result.metrics["criteria"]["peak_vertex_speed_m_s"], .02)

    def test_soft_slow_creep_cannot_accumulate_stable_duration(self):
        spec = build_spec("L2-06")
        e = OrangeEvaluator(spec)
        e.update(.1, soft_row(spec, lifted=True, touching=True))
        for i in range(1, 11):
            rows = soft_row(spec, at_goal=True)
            rows["orange"]["centroid"][0] += .0008 * i
            rows["orange"]["centroid_speed_m_s"] = .008
            self.assertEqual(e.update(.1 + i * .1, rows).status, "running")

    def test_soft_support_uses_collision_surface_and_rejects_nonfinite_motion(self):
        spec = build_spec("L2-06")
        for key in ("centroid_speed_m_s", "shape_speed_p95_m_s"):
            rows = soft_row(spec)
            rows["orange"][key] = float("nan")
            with self.assertRaises(InvalidSimulationState):
                OrangeEvaluator(spec).update(.1, rows)
        e = OrangeEvaluator(spec)
        e.update(.1, soft_row(spec, lifted=True, touching=True))
        rows = soft_row(spec, at_goal=True)
        rows["orange"]["collision_bounds_min"][2] += .01
        self.assertFalse(e.update(.2, rows).metrics["criteria"]["table_supported_geometry"])

    def test_drawer_release_near_threshold_can_finish_after_passive_travel(self):
        spec = build_spec("L2-07")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        row = rows["drawer"]
        row["fingers"]["left"] = [1, 1]
        e.update(.1, rows)
        row["pose"][1] += .074
        e.update(.2, rows)
        row["fingers"]["left"] = [0, 0]
        row["pose"][1] += .006
        e.update(.3, rows)
        self.assertEqual(e.update(.9, rows).status, "succeeded")

    def test_drawer_touch_or_small_oscillations_do_not_fake_a_grasped_pull(self):
        spec = build_spec("L2-07")
        for oscillate in (False, True):
            rows = measurement(spec)
            row = rows["drawer"]
            row["pose"][1] += .08
            start = row["pose"][1]
            e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
            row["fingers"]["left"] = [1, 1]
            for i in range(1, 15):
                row["pose"][1] = start + (.001 * (i % 2) if oscillate else 0)
                e.update(i * .1, rows)
            row["fingers"]["left"] = [0, 0]
            e.update(1.5, rows)
            self.assertEqual(e.update(2.2, rows).status, "running")

    def test_bowl_retains_supported_heap_but_not_airborne_or_spilled_pearl(self):
        spec = build_spec("L2-04")
        initial = {a.name: a.initial for a in spec.design.articles}
        rows = measurement(spec)
        for a in spec.design.articles:
            rows[a.name]["pose"] = list(a.goal)
        row = rows["load_602"]
        bowl = np.array(rows["bowl"]["pose"][:3])
        row["pose"][:3] = (bowl + [.006179, .014521, .054260]).tolist()
        weight = spec.parameters["ball_mass_kg"] * 9.81
        row["net_contact_force_world_n"] = [0, 0, weight]
        e = BenchEvaluator(spec, initial)
        e.flags.add("loaded_bowl_lifted")
        self.assertEqual(e.update(.1, rows).status, "running")
        result = e.update(.7, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["criteria"]["contained_balls"], 707)
        self.assertEqual(result.metrics["criteria"]["retained_balls"], 708)
        self.assertEqual(result.metrics["scoring_protocol"], "bowl_retained_load_v3")
        row["net_contact_force_world_n"] = [0, 0, 0]
        self.assertEqual(e.update(.8, rows).status, "running")
        row["net_contact_force_world_n"] = [0, 0, weight]
        row["linear_velocity"] = [0, 0, .2]
        self.assertEqual(e.update(.9, rows).status, "running")
        row["linear_velocity"] = [0, 0, 0]
        # A sphere well above the pile cannot pass just because it is slow.
        row["pose"][2] = bowl[2] + .09
        self.assertEqual(e.update(1., rows).status, "running")
        row["pose"][:3] = (bowl + [.09, 0, .01]).tolist()
        row["bench_force_n"] = weight
        e.update(1.1, rows)
        self.assertEqual(e.update(1.4, rows).status, "failed")
        # Returning a spilled pearl does not erase the recorded violation.
        row["pose"] = list(next(a for a in spec.design.articles if a.name == "load_602").goal)
        row["bench_force_n"] = 0
        self.assertEqual(e.update(2., rows).status, "failed")

    def test_bowl_internal_rolling_does_not_block_released_transport(self):
        spec = build_spec("L2-04")
        rows = measurement(spec)
        for article in spec.design.articles:
            rows[article.name]["pose"] = list(article.goal)
        rows["load_0"]["linear_velocity"] = [.04, 0., 0.]
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        e.flags.add("loaded_bowl_lifted")
        e.update(.1, rows)
        result = e.update(.7, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["criteria"]["unsettled_indices"], [0])
        # Low speed plus a NaN support measurement must never certify retention.
        rows["load_0"]["net_contact_force_world_n"] = [0., 0., float("nan")]
        with self.assertRaises(InvalidSimulationState):
            e.update(.8, rows)

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
        self.assertFalse(result.metrics["stages"]["released_on_table"])

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

    def test_native_inertia_with_adapters_preserves_mass_and_positive_eigenvalues(self):
        axes = SimpleNamespace(GetReal=lambda: 1.0, GetImaginary=lambda: [0, 0, 0])
        parts = [
            box("left", (0.04, 0.045, 0.02), (-0.084, 0, 0.03)),
            box("right", (0.04, 0.045, 0.02), (0.084, 0, 0.03)),
        ]
        mass, com, diagonal, quat = composite_inertia(
            0.12, [0, 0, 0.025], [0.0001, 0.0001, 0.0002], axes, parts, 0.016
        )
        self.assertAlmostEqual(mass, 0.136)
        self.assertGreater(min(diagonal), 0)
        np.testing.assert_allclose(com[:2], 0.0, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(quat), 1.0)

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

    def test_soft_initial_or_unreleased_or_hovering_is_not_success(self):
        s = build_spec("L2-06")
        e = OrangeEvaluator(s)
        for i in range(1, 10):
            self.assertEqual(e.update(i * 0.1, soft_row(s)).status, "running")
        e.update(1.0, soft_row(s, lifted=True, touching=True))
        self.assertEqual(
            e.update(1.1, soft_row(s, at_goal=True, touching=True)).status, "running"
        )
        self.assertEqual(
            e.update(1.2, soft_row(s, at_goal=True, lifted=True)).status, "running"
        )
        e.update(1.3, soft_row(s, at_goal=True))
        self.assertEqual(e.update(1.9, soft_row(s, at_goal=True)).status, "succeeded")

    def test_soft_sustained_deformation_stays_a_violation_after_shape_recovers(self):
        s = build_spec("L2-06")
        e = OrangeEvaluator(s)
        e.update(0.1, soft_row(s, lifted=True, touching=True, strain=0.4))
        early = e.update(0.4, soft_row(s, lifted=True, touching=True, strain=0.4))
        self.assertEqual(early.status, "failed")
        self.assertFalse(early.metrics["goal_complete"])
        e.update(0.5, soft_row(s, at_goal=True))
        result = e.update(1.1, soft_row(s, at_goal=True))
        self.assertEqual(result.status, "failed")
        self.assertTrue(result.metrics["goal_complete"])
        self.assertFalse(result.metrics["process_constraints_satisfied"])

    def test_soft_transient_deformation_not_irreversible_damage(self):
        s = build_spec("L2-06")
        e = OrangeEvaluator(s)
        e.update(0.1, soft_row(s, lifted=True, touching=True, strain=0.4))
        e.update(0.2, soft_row(s, at_goal=True))
        self.assertEqual(e.update(0.8, soft_row(s, at_goal=True)).status, "succeeded")

    def test_soft_nonfinite_and_stale_readback_rejected(self):
        s = build_spec("L2-06")
        e = OrangeEvaluator(s)
        e.update(0.1, soft_row(s))
        with self.assertRaises(InvalidSimulationState):
            e.update(0.1, soft_row(s))
        rows = soft_row(s)
        rows["orange"]["strain_p95"] = float("nan")
        with self.assertRaises(InvalidSimulationState):
            e.update(0.2, rows)


if __name__ == "__main__":
    unittest.main()
