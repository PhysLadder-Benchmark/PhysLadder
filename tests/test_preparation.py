"""Fixed startup pose must precede model observation and never be scored as GPT work."""

from dataclasses import replace
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from robot_tasks.backends.mock import MockBackend, MockRobot, MockTask
from robot_tasks.config import ExecutionSettings
from robot_tasks.registry import resolve_task
from robot_tasks.robots.preparation import preparation_action
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.runner import run_episode
from robot_tasks.types import Action, Assessment, Feedback


class PreparingRobot(MockRobot):
    def initialize(self, backend, config):
        super().initialize(backend, config)
        self.backend = backend
        self.applied = []
        self.finish_time = 0.0

    def apply(self, action):
        self.applied.append(action)
        self.finish_time = self.backend.time_s() + 0.25

    def feedback(self):
        return Feedback("completed" if self.backend.time_s() >= self.finish_time else "executing")


class StopPolicy:
    def __init__(self):
        self.seen = []

    def reset(self, seed):
        self.seen.clear()

    def act(self, context):
        self.seen.append(context)
        return Action((), 1 / 12, True)


class PreparationTests(unittest.TestCase):
    def setUp(self):
        _, self.config = resolve_task("L1-01")

    def test_ready_pose_is_fixed_robot_only_and_vertical(self):
        action = preparation_action(self.config)
        self.assertEqual(len(action.commands), 1)
        command = action.commands[0]
        self.assertEqual(command.kind, "cartesian_pose")
        np.testing.assert_allclose(command.values[:3], [0.36, 0.42, 0.25])
        np.testing.assert_allclose(pose_transform(command.values)[:3, 0], [0, 0, -1], atol=1e-12)
        changed = replace(
            self.config,
            seed=99,
            instruction="Different task",
            raw={**self.config.raw, "l1": {"position_jitter_m": 0.02}},
        )
        self.assertEqual(preparation_action(changed), action)

    def test_disable_and_invalid_execution_modes(self):
        raw = {**self.config.raw, "execution": {**self.config.raw["execution"], "startup_pose": "none"}}
        self.assertIsNone(preparation_action(replace(self.config, raw=raw)))
        raw["execution"]["startup_pose"] = "downward_ready_v1"
        raw["execution"]["motion_mode"] = "direct_ik"
        with self.assertRaisesRegex(ValueError, "requires"):
            ExecutionSettings.from_config(replace(self.config, raw=raw))

    def run_case(self, robot):
        policy = StopPolicy()
        with tempfile.TemporaryDirectory() as directory:
            path, result = run_episode(self.config, MockBackend(), robot, MockTask(), policy, Path(directory))
            events = [json.loads(l) for l in (path / "episode.jsonl").read_text().splitlines()]
        return result, events, policy

    def test_preparation_finishes_before_first_policy_observation(self):
        robot = PreparingRobot()
        result, events, policy = self.run_case(robot)
        self.assertEqual(len(robot.applied), 1)
        self.assertEqual(robot.applied[0], preparation_action(self.config))
        self.assertEqual(len(policy.seen), 1)
        complete = next(e for e in events if e["event"] == "preparation_complete")
        self.assertAlmostEqual(policy.seen[0].observation.time_s, complete["time_s"])
        self.assertEqual(policy.seen[0].observation.feedback.status, "completed")
        self.assertEqual(result["executed_actions"], 0)
        self.assertEqual(result["decisions"], 1)
        for event in events:
            if event.get("phase") == "preparation":
                self.assertEqual(event["evaluation"]["status"], "not_evaluated")

    def test_preparation_failure_never_calls_policy(self):
        class BlockedRobot(PreparingRobot):
            def feedback(self):
                return Feedback("executing")

        result, events, policy = self.run_case(BlockedRobot())
        self.assertEqual(policy.seen, [])
        self.assertEqual(result["status"], "error")
        self.assertFalse(result["task_evaluated"])
        self.assertIsNone(result["success"])
        self.assertIn("preparation", result["error"])

    def test_retry_repeats_preparation_before_next_policy_observation(self):
        class RetryPolicy(StopPolicy):
            def act(self, context):
                self.seen.append(context)
                return Action((), 1 / 12, len(self.seen) > 1)

        policy = RetryPolicy()

        class RetryTask(MockTask):
            resets = 0

            def reset(self, backend, seed):
                self.resets += 1

            def evaluate(self, state, feedback):
                if self.resets == 1 and policy.seen:
                    return Assessment("failed", "Synthetic retry trigger", retryable=True)
                return Assessment("running", "Fresh attempt")

        config = replace(self.config, raw={**self.config.raw, "execution": {
            **self.config.raw["execution"], "retry_failed_attempts": True}})
        robot = PreparingRobot()
        with tempfile.TemporaryDirectory() as directory:
            path, result = run_episode(config, MockBackend(), robot, RetryTask(), policy, Path(directory))
            events = [json.loads(line) for line in (path / "episode.jsonl").read_text().splitlines()]
        self.assertEqual(result["reset_count"], 1)
        self.assertEqual(len(policy.seen), 2)  # Policy history is retained across resets.
        self.assertEqual(robot.applied.count(preparation_action(config)), 2)
        completions = [e for e in events if e["event"] == "preparation_complete"]
        self.assertEqual(len(completions), 2)
        self.assertAlmostEqual(policy.seen[-1].observation.time_s, completions[-1]["time_s"])
        self.assertEqual(policy.seen[-1].observation.feedback.status, "attempt_reset")
        self.assertGreater(result["completed_steps"], result["final_attempt_time_s"] / config.control_dt_s)


if __name__ == "__main__":
    unittest.main()
