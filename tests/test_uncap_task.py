"""Offline contracts for table-pickup uncapping; no model/network/simulator calls."""

import unittest
from dataclasses import replace

from robot_tasks.policies.operating_notes import operating_notes
from robot_tasks.registry import resolve_task
from robot_tasks.robots.preparation import preparation_action
from robot_tasks.tasks.uncap.evaluation import UncapEvaluator
from robot_tasks.tasks.uncap.spec import SPEC, TARGETS
from robot_tasks.tasks.uncap.task import UncapTask
from robot_tasks.types import ArmState


def objects(separated=False, held=True):
    return {
        name: {
            "pose": [
                0.35,
                0.1,
                0.25
                + (
                    0
                    if name == "body"
                    else SPEC.cap_offset + (0.04 if separated else 0)
                ),
                1,
                0,
                0,
                0,
            ],
            "linear_velocity": [0, 0, 0],
            "angular_velocity": [0, 0, 0],
            "fingers": {
                arm: ([1.0, 1.0] if held and arm == owner else [0.0, 0.0])
                for arm in ("left", "right")
            },
            "bench_force_n": 0.0,
            "pair_force_n": 0.0 if separated else 0.5,
        }
        for name, owner in (("body", "left"), ("cap", "right"))
    }


def placed():
    rows = objects(separated=True, held=False)
    # Both lie along world X, inside their own zones; axis signs preserve open-end clearance.
    for name, row in rows.items():
        row["pose"] = [
            *TARGETS[name],
            0.025,
            0.7071067811865476,
            0,
            0.7071067811865476,
            0,
        ]
        row["bench_force_n"] = 0.5
    # Put cap slightly farther along X to keep its opening fully clear of the body tip.
    rows["body"]["pose"][0] -= 0.035
    rows["cap"]["pose"][0] += 0.035
    return rows


class UncapTests(unittest.TestCase):
    def setUp(self):
        self.e = UncapEvaluator()
        self.robot = {
            arm: ArmState(
                gripper_width_m=0.07, tcp_pose_world_wxyz=(0.1, 0.1, 0.4, 1, 0, 0, 0)
            )
            for arm in ("left", "right")
        }

    def test_configuration_is_two_manipulation_arms_with_shared_workflow(self):
        spec, cfg = resolve_task("L2-01")
        cfg = spec.factory.prepare_config(cfg)
        self.assertEqual(cfg.arms, ("left", "right"))
        self.assertEqual(cfg.observation_arms, ())
        self.assertEqual(cfg.grasp_check_mode, "contact_inspection")
        self.assertEqual(cfg.raw["execution"]["max_decisions"], 100)
        self.assertEqual(cfg.raw["policy"]["reasoning_effort"], "medium")
        self.assertIn(
            "Both hands start empty", operating_notes(cfg)["startup_preparation"]
        )
        self.assertNotIn(
            "during a short lift", operating_notes(cfg)["manipulation_guidance"]
        )
        self.assertEqual(preparation_action(cfg).commands, ())
        self.assertFalse(hasattr(UncapTask, "initialize_episode"))
        with self.assertRaises(ValueError):
            UncapTask.prepare_config(replace(cfg, arms=("left",)))

    def test_policy_guidance_does_not_require_free_vertical_lift(self):
        from robot_tasks.policies.gpt import GPTPolicy
        from robot_tasks.policies.model_config import GPTSettings

        _, cfg = resolve_task("L2-01")
        policy = GPTPolicy(
            UncapTask.prepare_config(cfg),
            GPTSettings("https://example.org", "fixture-model"),
        )
        self.assertIn(
            "free-object vertical test-lift guard is NOT applicable",
            policy.instructions,
        )
        self.assertNotIn("Until status=verified", policy.instructions)
        self.assertNotIn("Observation arms (", policy.instructions)

    def test_optional_task_initialization_hook_precedes_policy_observation(self):
        import tempfile
        from pathlib import Path

        from fixtures import rgbd

        from robot_tasks.backends.mock import MockBackend, MockRobot, MockTask
        from robot_tasks.runner import run_episode
        from robot_tasks.types import Action

        order = []

        class Backend(MockBackend):
            def capture_rgb(self):
                return tuple(rgbd(name, self.t) for name in self.config.cameras)

        class Task(MockTask):
            def initialize_episode(self, backend, robot, tick):
                order.append("initialize")
                tick(phase="task_initialization", score=False)

            def evaluate(self, state, feedback):
                assert order and order[0] == "initialize"
                return super().evaluate(state, feedback)

        class Policy:
            def reset(self, seed):
                pass

            def act(self, context):
                order.append("policy")
                return Action((), cfg.control_dt_s)

        _, cfg = resolve_task("L2-01")
        cfg = UncapTask.prepare_config(cfg)
        cfg = replace(cfg, raw={**cfg.raw, "execution": {"max_decisions": 1}})
        with tempfile.TemporaryDirectory() as directory:
            _, result = run_episode(
                cfg, Backend(), MockRobot(), Task(), Policy(), Path(directory)
            )
        self.assertEqual(order, ["initialize", "policy"])
        self.assertIsNone(result["error"])
        self.assertEqual(result["grasp_check_protocol"], "contact_inspection")

    def test_gripper_fits_both_parts(self):
        _, cfg = resolve_task("L2-01")
        lower, upper = cfg.robot.gripper.width_limits_m
        self.assertTrue(lower < 2 * SPEC.body_radius < upper)
        self.assertTrue(lower < 2 * SPEC.cap_radius < upper)
        self.assertGreater(SPEC.overlap, 0)
        self.assertGreater(SPEC.sleeve_interference, 0)

    def test_separation_without_two_real_grasps_is_not_success(self):
        self.e.update(0, objects(held=False), self.robot)
        result = self.e.update(1, objects(separated=True, held=False), self.robot)
        self.assertFalse(result.metrics["separated_while_held"])
        for t in (2, 3, 4):
            result = self.e.update(t, placed(), self.robot)
        self.assertNotEqual(result.status, "succeeded")

    def test_held_separation_then_release_and_stability(self):
        self.e.update(0, objects(), self.robot)
        result = self.e.update(0.1, objects(separated=True), self.robot)
        self.assertTrue(result.metrics["separated_while_held"])
        self.assertEqual(result.status, "running")
        # Scoring of the final separated layout uses actual mouth clearance.
        rows = placed()
        rows["cap"]["pose"][0] += 0.025
        self.assertEqual(self.e.update(0.2, rows, self.robot).status, "running")
        self.assertEqual(self.e.update(0.7, rows, self.robot).status, "running")
        self.assertEqual(self.e.update(1.2, rows, self.robot).status, "succeeded")

    def test_short_release_descent_can_settle_and_complete(self):
        self.e.update(0, objects(separated=True), self.robot)
        rows = objects(separated=True, held=False)
        rows["cap"]["linear_velocity"][2] = -0.2
        self.assertEqual(self.e.update(0.1, rows, self.robot).status, "running")
        self.assertEqual(self.e.update(0.2, placed(), self.robot).status, "running")
        self.assertEqual(self.e.update(1.3, placed(), self.robot).status, "succeeded")

    def test_force_reference_is_advisory_not_damage_claim(self):
        rows = objects()
        rows["cap"]["fingers"]["right"] = [26, 1]
        result = self.e.update(0, rows, self.robot)
        self.assertEqual(result.status, "running")
        self.assertTrue(result.metrics["force_reference_exceeded"])
        self.assertTrue(result.metrics["quality_warnings"])
        self.assertEqual(
            result.metrics["damage_model"], "none; force limits are protocol proxies"
        )
        self.e.update(0.1, objects(separated=True), self.robot)
        self.e.update(0.2, placed(), self.robot)
        result = self.e.update(1.3, placed(), self.robot)
        self.assertEqual(result.status, "succeeded")
        self.assertTrue(result.metrics["quality_warnings"])

    def test_part_lost_below_table_ends_episode(self):
        rows = objects(separated=True, held=False)
        rows["cap"]["pose"][2] = -0.2
        result = self.e.update(0, rows, self.robot)
        self.assertEqual(result.status, "failed")
        self.assertFalse(result.retryable)

    def test_placement_outside_colored_zones_is_allowed(self):
        self.e.update(0, objects(separated=True), self.robot)
        rows = placed()
        rows["body"]["pose"][1] = 0.05
        for t in (1, 2, 3):
            result = self.e.update(t, rows, self.robot)
        self.assertEqual(result.status, "succeeded")

    def test_still_held_on_table_is_not_placement(self):
        self.e.update(0, objects(separated=True), self.robot)
        rows = placed()
        rows["body"]["fingers"]["left"] = [1, 1]
        for t in (1, 2, 3):
            self.assertNotEqual(self.e.update(t, rows, self.robot).status, "succeeded")

    def test_no_minimum_spacing_between_placed_parts(self):
        self.e.update(0, objects(separated=True), self.robot)
        rows = placed()
        rows["body"]["pose"][:3] = [0.35, 0.1, SPEC.body_radius]
        rows["cap"]["pose"][:3] = [0.35, 0.139, SPEC.cap_radius]
        for t in (1, 2):
            result = self.e.update(t, rows, self.robot)
        self.assertEqual(result.status, "succeeded")

    def test_putting_cap_back_is_not_uncapped_completion(self):
        self.e.update(0, objects(separated=True), self.robot)
        rows = objects(separated=False, held=False)
        for row in rows.values():
            row["bench_force_n"] = 0.5
        for t in (1, 2):
            result = self.e.update(t, rows, self.robot)
        self.assertTrue(result.metrics["recapped"])
        self.assertNotEqual(result.status, "succeeded")

    def test_monotonic_sample_time(self):
        self.e.update(0, objects(), self.robot)
        with self.assertRaises(ValueError):
            self.e.update(0, objects(), self.robot)


if __name__ == "__main__":
    unittest.main()
