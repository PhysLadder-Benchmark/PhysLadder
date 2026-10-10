"""Goal regressions for the simplified pedestal lift; no model calls."""

import unittest
from copy import deepcopy

from test_native_l2_scoring import soft_row
from robot_tasks.registry import resolve_task
from robot_tasks.tasks.bench.orange_evaluation import OrangeEvaluator
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.types import InvalidSimulationState


class OrangeLiftTests(unittest.TestCase):
    def setUp(self):
        self.spec = build_spec("L2-06")
        self.evaluator = OrangeEvaluator(self.spec)
        self.initial = soft_row(self.spec)
        self.first = self.evaluator.update(0.0, self.initial)

    def sample(self, rise, touching=True):
        rows = deepcopy(self.initial)
        row = rows["orange"]
        for k in (
            "bounds_min",
            "bounds_max",
            "centroid",
            "collision_bounds_min",
            "collision_bounds_max",
        ):
            row[k][2] += rise
        if touching:
            row["finger_distances_m"][:2] = [0.001, 0.001]
        return rows

    def test_initial_pedestal_height_does_not_count_as_lift(self):
        self.assertEqual(self.first.status, "running")
        self.assertGreater(self.first.metrics["criteria"]["initial_centroid_z_m"], 0.91)
        self.assertEqual(self.first.metrics["criteria"]["centroid_rise_m"], 0)
        self.assertEqual(self.evaluator.update(0.1, self.sample(0)).status, "running")

    def test_50mm_threshold_finishes_immediately_while_held(self):
        self.assertEqual(
            self.evaluator.update(0.1, self.sample(0.049)).status, "running"
        )
        result = self.evaluator.update(0.2, self.sample(0.05))
        self.assertEqual(result.status, "succeeded")
        self.assertFalse(result.metrics["criteria"]["release_required"])
        self.assertEqual(result.metrics["criteria"]["hold_s"], 0)

    def test_flying_or_one_finger_touch_is_not_a_held_lift(self):
        for distances in ([0.1, 0.1], [0.001, 0.1], [0.1, 0.001]):
            row = self.sample(0.07)
            row["orange"]["finger_distances_m"][:2] = distances
            self.assertEqual(
                self.evaluator.update(self.evaluator.last_time + 0.1, row).status,
                "running",
            )

    def test_sideways_transport_is_unnecessary_but_does_not_fake_height(self):
        rows = self.sample(0)
        rows["orange"]["centroid"][0] += 0.2
        self.assertEqual(self.evaluator.update(0.1, rows).status, "running")
        rows = self.sample(0.06)
        rows["orange"]["centroid"][0] += 0.2
        self.assertEqual(self.evaluator.update(0.2, rows).status, "succeeded")

    def test_speed_deformation_and_local_inversion_are_diagnostic_only(self):
        rows = self.sample(0.06)
        rows["orange"].update(
            speed_m_s=2.0,
            centroid_speed_m_s=0.2,
            shape_speed_p95_m_s=0.5,
            strain_p95=0.8,
            strain_max=2.0,
            minimum_volume_ratio=-0.1,
        )
        result = self.evaluator.update(0.1, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertTrue(result.metrics["quality_warnings"])

    def test_remaining_on_support_does_not_count_as_lift(self):
        rows = self.sample(0.06)
        rows["orange"]["collision_bounds_min"][2] = self.spec.parameters["support_z"]
        self.assertEqual(self.evaluator.update(0.1, rows).status, "running")

    def test_fallen_onto_table_can_retry_but_below_table_fails(self):
        self.assertEqual(
            self.evaluator.update(0.1, self.sample(-0.13, False)).status, "running"
        )
        self.assertEqual(
            self.evaluator.update(0.2, self.sample(-0.25, False)).status, "failed"
        )

    def test_pause_does_not_reset_reference_or_create_progress(self):
        self.assertIs(self.evaluator.update(0.0, self.sample(0.06)), self.first)
        result = self.evaluator.update(0.1, self.sample(0.05))
        self.assertEqual(result.status, "succeeded")

    def test_invalid_clock_and_measurements_remain_framework_errors(self):
        for t in (-1.0, float("nan"), float("inf")):
            with self.assertRaises(InvalidSimulationState):
                self.evaluator.update(t, self.initial)
        rows = self.sample(0.06)
        rows["orange"]["centroid"][2] = float("nan")
        with self.assertRaises(InvalidSimulationState):
            self.evaluator.update(0.1, rows)

    def test_task_prompt_physics_and_goal_agree(self):
        entry, cfg = resolve_task("L2-06")
        cfg = entry.factory.prepare_config(cfg)
        self.assertEqual(cfg.raw["orange_physics"]["dynamic_friction"], 2.0)
        self.assertEqual(
            cfg.raw["task"]["scoring_protocol"],
            self.spec.parameters["scoring_protocol"],
        )
        self.assertEqual(cfg.arms, ("left",))
        self.assertEqual(cfg.observation_arms, ("right",))
        self.assertFalse(self.spec.design.release)
        self.assertFalse(self.spec.design.fixtures)
        self.assertIn("at least 5 cm", cfg.instruction)
        self.assertAlmostEqual(self.spec.design.articles[0].initial[2], 0.912)
