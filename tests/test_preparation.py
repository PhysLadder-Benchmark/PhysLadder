"""Fixed startup pose must precede model observation and never be scored as GPT work."""

from dataclasses import replace
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from robot_tasks.backends.mock import MockRobot, MockTask
from robot_tasks.config import ExecutionSettings
from robot_tasks.workflow import workflow_metadata
from robot_tasks.registry import resolve_task
from robot_tasks.robots.preparation import preparation_action
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.cameras.profile import load_camera_rig
from robot_tasks.cameras.geometry import project_world
from robot_tasks.runner import run_episode
from robot_tasks.types import Action, Assessment, Feedback
from fixtures import RGBDBackend


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
        self.assertEqual(len(action.commands), 2)
        command = action.commands[0]
        self.assertEqual(command.arm, "left")
        self.assertEqual(self.config.observation_arms, ("right",))
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

    def camera_pose(self, config, action, spec):
        command = next(c for c in action.commands if c.arm == spec.parent)
        return (pose_transform(command.values) @ np.linalg.inv(pose_transform(config.robot.tcp_pose_wxyz))
                @ pose_transform(spec.optical_pose_wxyz))

    def test_observer_camera_faces_work_area_and_moves_inward(self):
        from robot_tasks.robots.transforms import transform_pose
        rig = load_camera_rig(self.config.source.parent / self.config.raw["simulation"]["camera_profile"])
        for active, observer in (("left", "right"), ("right", "left")):
            with self.subTest(observer=observer):
                config = replace(self.config, arms=(active,), observation_arms=(observer,))
                action = preparation_action(config)
                self.assertTrue(all(c.kind == "cartesian_pose" for c in action.commands))
                spec = next(c for c in rig.cameras if c.parent == observer)
                camera = self.camera_pose(config, action, spec)
                focus = np.array(action.commands[0].values[:3]) - (0.0, 0.0, 0.12)
                pixel = project_world([focus], transform_pose(camera), spec.intrinsics)[0]
                np.testing.assert_allclose(pixel, spec.intrinsics[2:], atol=1e-8)
                np.testing.assert_allclose(camera[:3, 3], [.28, -.10 if observer == "right" else .10, .50], atol=1e-8)
                np.testing.assert_allclose(camera[:3, 0] @ [0, 0, 1], 0.0, atol=1e-8)

    def test_camera_mount_change_updates_tcp_goal_and_preserves_view(self):
        rig = load_camera_rig(self.config.source.parent / self.config.raw["simulation"]["camera_profile"])
        original = next(c for c in rig.cameras if c.parent == "right")
        moved = replace(original, optical_pose_wxyz=(.10, .02, .08, 1., 0., 0., 0.))
        new_rig = replace(rig, cameras=tuple(moved if c is original else c for c in rig.cameras))
        before = preparation_action(self.config)
        with patch("robot_tasks.robots.preparation.load_camera_rig", return_value=new_rig):
            after = preparation_action(self.config)
        self.assertNotEqual(before.commands[-1].values, after.commands[-1].values)
        np.testing.assert_allclose(self.camera_pose(self.config, before, original),
                                   self.camera_pose(self.config, after, moved), atol=1e-8)

    def test_only_declared_observation_arms_get_camera_preparation(self):
        for arms in (("left",), ("left", "right")):
            config = replace(self.config, arms=arms, observation_arms=())
            with patch("robot_tasks.robots.preparation.load_camera_rig") as load:
                action = preparation_action(config)
            load.assert_not_called()
            self.assertEqual(tuple(c.arm for c in action.commands), arms)
            for command in action.commands:
                np.testing.assert_allclose(pose_transform(command.values)[:3, 0], [0, 0, -1], atol=1e-12)

    def test_missing_observer_camera_fails_before_startup(self):
        config = replace(self.config, cameras=("overview", "left_wrist"))
        with self.assertRaisesRegex(ValueError, "exactly one enabled wrist camera"):
            preparation_action(config)

    def test_preparation_uses_shared_executor_without_task_switches(self):
        self.assertEqual(workflow_metadata()["startup_preparation"], "downward_ready_v1")
        for field, value in (("startup_pose", "none"), ("motion_mode", "direct_ik")):
            raw = {**self.config.raw, "execution": {**self.config.raw["execution"], field: value}}
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "Unknown execution"):
                ExecutionSettings.from_config(replace(self.config, raw=raw))

    def run_case(self, robot):
        policy = StopPolicy()
        with tempfile.TemporaryDirectory() as directory:
            path, result = run_episode(self.config, RGBDBackend(), robot, MockTask(), policy, Path(directory),
                                       preparation=preparation_action(self.config))
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
            path, result = run_episode(config, RGBDBackend(), robot, RetryTask(), policy, Path(directory),
                                       preparation=preparation_action(config))
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
