"""Episode validity and observer-only roles; entirely offline, no model calls."""

import unittest

from test_native_l2_scoring import soft_row
from test_single_episode import SingleEpisodeTests, TerminalTask, TrialPolicy

from robot_tasks.tasks.bench.observer_contact import ObserverContactMonitor
from robot_tasks.tasks.bench.orange_evaluation import OrangeEvaluator
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.types import Assessment, InvalidSimulationState


class InvalidTask(TerminalTask):
    def __init__(self, *, early=False, exception=False):
        super().__init__()
        self.early, self.exception = early, exception

    def evaluate(self, state, feedback):
        if self.early or state.values["simulation_time_s"] > 0:
            if self.exception:
                raise InvalidSimulationState("Nonfinite measured state")
            return Assessment(
                "invalid", "Invalid FEM", {"invalid_reason": "simulation_invalid"}
            )
        return Assessment("running", "Initial state", {"stages": {"grasped": True}})


class ValidityTests(unittest.TestCase):
    setUp = SingleEpisodeTests.setUp
    run_case = SingleEpisodeTests.run_case

    def test_invalid_measurements_end_without_scoring_model_failure(self):
        for early in (False, True):
            for exception in (False, True):
                with self.subTest(early=early, exception=exception):
                    result = self.run_case(
                        InvalidTask(early=early, exception=exception), TrialPolicy()
                    )
                    self.assertEqual(result["status"], "invalid")
                    self.assertEqual(result["evaluation_validity"], "invalid")
                    self.assertIsNone(result["success"])
                    self.assertFalse(result["task_evaluated"])
                    self.assertEqual(result["model_calls"], 0 if early else 1)
                    if exception and not early:
                        self.assertTrue(
                            result["assessment"]["metrics"]["stages"]["grasped"]
                        )


class ObserverTests(unittest.TestCase):
    def test_all_five_l2_runtime_prompts_disclose_information_and_control_scope(self):
        from robot_tasks.registry import resolve_task
        from robot_tasks.tasks.bench.l2_protocol import OPERATING_NOTES

        for task_id in ("L2-01", "L2-03", "L2-04", "L2-06", "L2-07"):
            spec, config = resolve_task(task_id)
            config = spec.factory.prepare_config(config)
            self.assertEqual(config.instruction.count(OPERATING_NOTES), 1)
            self.assertEqual(config.raw["task"]["prompt_protocol"], "guided_rgbd_l2_v2")
            self.assertIn("no direct force, torque", config.instruction)

    def test_transient_contact_does_not_fail_but_sustained_contact_does(self):
        monitor = ObserverContactMonitor()

        def update(t, force):
            return monitor.update(
                t,
                {"right": {"mode": "rigid_contact", "max_force_n": force}},
                Assessment("running", "grasping", {"stages": {"grasped": True}}),
            )

        self.assertEqual(update(0.1, 1).status, "running")
        self.assertEqual(update(0.2, 0).status, "running")
        self.assertEqual(update(0.3, 1).status, "running")
        result = update(0.6, 1)
        self.assertEqual(result.status, "failed")
        self.assertEqual(result.metrics["failure_category"], "observer_role_violation")
        self.assertTrue(result.metrics["stages"]["grasped"])

    def test_soft_bounds_nearness_requires_review_not_fake_contact_failure(self):
        monitor = ObserverContactMonitor()
        evidence = {"right": {"mode": "soft_geometry_proximity", "distance_m": 0.002}}
        monitor.update(0.1, evidence, Assessment("running", ""))
        result = monitor.update(0.4, evidence, Assessment("succeeded", ""))
        self.assertEqual(result.status, "invalid")
        self.assertEqual(
            result.metrics["invalid_reason"], "observer_contact_requires_review"
        )
        self.assertFalse(
            result.metrics["observer_contact"]["geometry_is_contact_proof"]
        )

    def test_numeric_invalid_takes_precedence_over_contact(self):
        monitor = ObserverContactMonitor()
        evidence = {"right": {"mode": "rigid_contact", "max_force_n": 1}}
        monitor.update(0.1, evidence, Assessment("running", ""))
        result = monitor.update(0.4, evidence, Assessment("invalid", "FEM inversion"))
        self.assertEqual(result.reason, "FEM inversion")

    def test_inverted_soft_element_is_not_tissue_damage_or_model_failure(self):
        spec = build_spec("L2-06")
        rows = soft_row(spec, strain=0.8)
        rows["orange"]["minimum_volume_ratio"] = -0.1
        result = OrangeEvaluator(spec).update(0.1, rows)
        self.assertEqual(result.status, "invalid")
        self.assertEqual(result.metrics["invalid_reason"], "simulation_invalid")


if __name__ == "__main__":
    unittest.main()
