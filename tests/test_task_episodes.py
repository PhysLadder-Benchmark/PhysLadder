"""Offline regression tests for target execution, honest scoring and isolation."""

from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import replace
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from robot_tasks.backends.mock import MockBackend, MockRobot, MockTask, NoopPolicy
from robot_tasks.config import ExecutionSettings, load_config
from robot_tasks.integrationtest.gpt_check import MotionGoal
from robot_tasks.policies.gpt import GPTPolicy
from robot_tasks.policies.model_config import GPTSettings
from robot_tasks.policies.responses import ModelReply
from robot_tasks.runner import run_episode
from robot_tasks.tasks.pick_place import PickPlaceEvaluator, SingleArmPickPlace
from robot_tasks.tasks.pick_place_scene import InteractionSettings
from robot_tasks.types import Action, ArmCommand, Assessment, Feedback

ROOT = Path(__file__).resolve().parents[1]


class PickPlaceScoreTests(unittest.TestCase):
    def setUp(self):
        self.settings = InteractionSettings()
        self.source = (0.38, 0.42, 0.02)
        self.target = (0.38, 0.27, 0.02)
        self.scorer = PickPlaceEvaluator(self.settings, self.source, self.target)

    def sample(self, t, *, lifted=False, target=False, released=False):
        return {"time_s": t, "object": {
            "position_m": [0.38, 0.27 if target else 0.42, 0.12 if lifted else 0.02],
            "linear_velocity_m_s": [0., 0., 0.], "angular_velocity_rad_s": [0., 0., 0.],
            "finger_normal_force_n": [0., 0.] if released else [2., 2.]},
            "tcp_pose_world_wxyz": [0.38, 0.27, 0.18, 1., 0., 0., 0.],
            "gripper_width_m": 0.07 if released else 0.04}

    def lift(self):
        for t in (0., 0.5, 1.):
            score = self.scorer.update(self.sample(t, lifted=True))
        self.assertTrue(score.metrics["lift_confirmed"])

    def test_real_lift_then_release_and_stability_succeeds(self):
        self.lift()
        for t in (1.5, 2., 2.5):
            score = self.scorer.update(self.sample(t, target=True, released=True))
        self.assertEqual(score.status, "succeeded")

    def test_pushing_to_target_without_lifting_never_succeeds(self):
        for t in (0., 0.5, 1., 1.5):
            score = self.scorer.update(self.sample(t, target=True, released=True))
        self.assertEqual(score.status, "running")
        self.assertFalse(score.metrics["lift_confirmed"])

    def test_lift_without_release_and_momentary_placement_are_not_success(self):
        self.lift()
        for t in (1.5, 2., 2.5):
            score = self.scorer.update(self.sample(t, target=True))
        self.assertEqual(score.status, "running")
        self.assertEqual(self.scorer.update(self.sample(3., target=True, released=True)).status, "running")

    def test_drop_is_recorded_and_fall_below_table_fails(self):
        self.lift()
        falling = self.sample(1.5, lifted=True, released=True)
        falling["object"]["linear_velocity_m_s"][2] = -0.2
        score = self.scorer.update(falling)
        self.assertEqual(score.metrics["drop_events"], 1)
        falling = deepcopy(falling)
        falling["time_s"] = 2.
        falling["object"]["position_m"][2] = -0.1
        self.assertEqual(self.scorer.update(falling).status, "failed")

    def test_explicit_thresholds_change_score(self):
        self.settings = replace(self.settings, placement_tolerance_m=0.01)
        self.scorer = PickPlaceEvaluator(self.settings, self.source, self.target)
        self.lift()
        for t in (1.5, 2., 2.5):
            row = self.sample(t, target=True, released=True)
            row["object"]["position_m"][0] += 0.02
            score = self.scorer.update(row)
        self.assertEqual(score.status, "running")


class TrackingRobot(MockRobot):
    def initialize(self, backend, config):
        super().initialize(backend, config)
        self.backend = backend
        self.applied = []
        self.target_time = None

    def reset(self, seed):
        self.applied.clear()
        self.target_time = None

    def apply(self, action):
        self.applied.append(action)
        self.target_time = self.backend.time_s() + 0.3

    def feedback(self):
        done = self.target_time is None or self.backend.time_s() >= self.target_time - 1e-9
        return Feedback("completed" if done else "executing")


class RunningTask(MockTask):
    def evaluate(self, state, feedback):
        return Assessment("running", "Not finished", {"hidden_target": [123., 456.]})


class EpisodeExecutionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        # Runtime snapshots use resolved paths, even if Windows TEMP is an alias.
        self.root = Path(self.temp.name).resolve()
        base = load_config(ROOT / 'configs/tasks/l1_01_cylinder_insertion.toml')
        self.config = replace(base, control_dt_s=0.1, raw={**base.raw, "policy_options": {}, "execution": {
            "max_action_steps": 8, "completion_samples": 2, "wait_steps": 3, "max_decisions": 4}})

    def run_case(self, policy, *, config=None, robot=None):
        backend = MockBackend()
        robot = robot or TrackingRobot()
        with redirect_stdout(io.StringIO()):
            path, result = run_episode(config or self.config, backend, robot, RunningTask(), policy, self.root)
        return path, result, backend, robot

    def test_one_target_is_held_until_completed_before_next_decision(self):
        contexts = []
        class Policy(NoopPolicy):
            def act(policy, context):
                contexts.append(context)
                return Action((ArmCommand("left", "gripper", (0.04,)),) if context.step == 1 else (),
                              policy.duration_s, request_stop=context.step > 1)
        _, result, backend, robot = self.run_case(Policy(0.1))
        self.assertEqual(len(robot.applied), 1)
        self.assertAlmostEqual(contexts[1].observation.time_s, 0.4)
        self.assertEqual(contexts[1].observation.feedback.status, "completed")
        self.assertNotIn("hidden_target", repr(contexts))
        self.assertEqual(result["success"], False)
        self.assertTrue(result["false_completion"])
        self.assertTrue(backend.closed)

    def test_blocked_action_returns_control_without_waiting_for_timeout(self):
        seen = []

        class BlockedRobot(TrackingRobot):
            def feedback(robot):
                if robot.applied:
                    return Feedback("blocked", "Opening stalled; queued motion cancelled.",
                                    {"reason_code": "opening_stalled"})
                return super().feedback()

        class Policy(NoopPolicy):
            def act(policy, context):
                seen.append(context)
                return Action((ArmCommand("left", "gripper", (0.07,)),) if context.step == 1 else (),
                              policy.duration_s, request_stop=context.step > 1)

        _, result, _, robot = self.run_case(Policy(0.1), robot=BlockedRobot())
        self.assertEqual(seen[1].observation.feedback.status, "blocked")
        self.assertAlmostEqual(seen[1].observation.time_s, self.config.control_dt_s)
        self.assertEqual(result["action_timeouts"], 0)
        self.assertFalse(result["success"])
        self.assertEqual(len(robot.applied), 1)

    def test_action_timeout_is_returned_to_policy_without_resetting_scene(self):
        seen = []
        config = replace(self.config, raw={**self.config.raw, "execution": {"max_action_steps": 1}})
        class Policy(NoopPolicy):
            def act(policy, context):
                seen.append(context)
                return Action((ArmCommand("left", "gripper", (0.04,)),) if context.step == 1 else (),
                              policy.duration_s, request_stop=context.step > 1)
        _, result, _, _ = self.run_case(Policy(0.1), config=config)
        self.assertEqual(seen[1].observation.feedback.status, "action_timeout")
        self.assertEqual(result["action_timeouts"], 1)
        self.assertEqual(result["status"], "finished")

    def test_ik_rejection_returns_feedback_and_allows_a_fresh_model_action(self):
        class RejectOnce(TrackingRobot):
            def apply(robot, action):
                if not hasattr(robot, "refused"):
                    robot.refused = True
                    robot.rejection = True
                    raise ValueError("Unreachable target")
                robot.rejection = False
                super().apply(action)

            def feedback(robot):
                if getattr(robot, "rejection", False):
                    return Feedback("rejected", "Unreachable target")
                return super().feedback()

        seen = []
        class Policy(NoopPolicy):
            def act(policy, context):
                seen.append(context)
                return Action((ArmCommand("left", "gripper", (0.04,)),) if context.step < 3 else (),
                              policy.duration_s, request_stop=context.step >= 3)
        _, result, _, robot = self.run_case(Policy(0.1), robot=RejectOnce())
        self.assertEqual(seen[1].observation.feedback.status, "rejected")
        self.assertEqual(seen[1].observation.time_s, 0.)
        self.assertEqual(len(robot.applied), 1)
        self.assertEqual(result["rejected_actions"], 1)
        self.assertEqual(result["executed_actions"], 1)
        self.assertEqual(result["status"], "finished")

    def test_final_observation_and_budget_exhaustion_are_scored_failures(self):
        config = replace(self.config, max_steps=4, raw={**self.config.raw, "execution": {
            "max_action_steps": 4, "wait_steps": 2, "stop_steps": 3}})
        class Policy(NoopPolicy):
            def act(policy, context):
                return Action((), policy.duration_s, request_stop=context.step > 1)
        _, result, _, _ = self.run_case(Policy(0.1), config=config)
        self.assertEqual(result["termination_reason"], "max_steps")
        self.assertEqual(result["completed_steps"], 4)
        self.assertTrue(result["task_evaluated"])
        self.assertFalse(result["success"])

    def test_late_model_response_never_executes(self):
        clock = [0.0]
        config = replace(self.config, max_wall_time_s=1.)
        class Policy(NoopPolicy):
            def act(policy, context):
                clock[0] = 2.
                return Action((ArmCommand("left", "gripper", (0.04,)),), policy.duration_s)
        with patch("robot_tasks.runner.time.monotonic", side_effect=lambda: clock[0]):
            _, result, _, robot = self.run_case(Policy(0.1), config=config)
        self.assertFalse(robot.applied)
        self.assertEqual(result["termination_reason"], "wall_time_limit")
        self.assertFalse(result["success"])

    def test_gpt_history_and_outputs_reset_even_if_policy_object_is_reused(self):
        settings = GPTSettings("https://example.org/openai", "gpt-6-astra")
        client = Mock()
        stop = '{"commands":[],"request_stop":true}'
        client.generate.return_value = ModelReply(stop, settings.model, "fixture", {}, 0.)
        policy = GPTPolicy(self.config, settings, client=client)
        first, result1, _, _ = self.run_case(policy)
        second, result2, _, _ = self.run_case(policy)
        self.assertNotEqual(first, second)
        self.assertEqual(result1["model_calls"], 1)
        self.assertEqual(result2["model_calls"], 1)
        for call in client.generate.call_args_list:
            self.assertEqual(len(call.kwargs["messages"]), 1)
        self.assertEqual(result1["simulation_time_s"], result2["simulation_time_s"])
        self.assertEqual(result1["initial_robot"], result2["initial_robot"])

    def test_pick_place_disables_right_actions_and_formats_only_public_goals(self):
        from robot_tasks.policies.action_codec import decode_action
        config = SingleArmPickPlace.prepare_config(load_config(ROOT / "configs/tasks/gpt_single_arm_pick_place.toml"))
        self.assertEqual(config.arms, ("left",))
        self.assertNotIn("{", config.instruction)
        self.assertNotIn("0.38", config.instruction)
        with self.assertRaisesRegex(ValueError, "Unknown arm"):
            decode_action('{"commands":[{"arm":"right","kind":"gripper","values":[0.06],"frame":"world"}],"request_stop":false}', config)

    def test_shared_episode_records_runtime_without_a_windows_manifest(self):
        local = self.root / "runtime.toml"
        local.write_text('[paths]\ncache_dir="cache"\noutput_dir="episodes"', encoding="utf-8")
        with patch.dict(os.environ, {"GPT_BENCHMARK_RUNTIME_CONFIG": str(local)}), \
                patch("robot_tasks.runtime.STORAGE_CONFIG", self.root / "missing_windows.json"):
            path, result, _, _ = self.run_case(NoopPolicy(self.config.control_dt_s))
        self.assertEqual(result["status"], "finished")
        runtime = json.loads((path / "runtime_config.json").read_text())
        storage = json.loads((path / "storage_config.json").read_text())
        self.assertEqual(runtime["python"], sys.executable)
        self.assertEqual(runtime["cache_dir"], str(self.root / "cache"))
        self.assertEqual(runtime["storage_mode"], "portable")
        self.assertFalse(storage["windows_storage_configured"])

    def test_motion_goal_and_prompt_share_changed_parameters(self):
        config = load_config(ROOT / "configs/checks/r5_gpt.toml")
        config = replace(config, raw={**config.raw, "motion_goal": {**config.raw["motion_goal"],
                                                                   "lift_delta_m": 0.035, "gripper_width_m": 0.05}})
        goal = MotionGoal.from_config(config)
        self.assertIn("0.035 metres", goal.instruction(config.instruction))
        self.assertIn("0.05 metres", goal.instruction(config.instruction))

    def test_action_and_ik_use_the_same_quaternion_norm_tolerance(self):
        from robot_tasks.policies.action_codec import decode_action
        from robot_tasks.robots.transforms import pose_transform
        for scalar in (0.9994, 1.0006):
            pose = [0.3, 0.4, 0.2, scalar, 0., 0., 0.]
            document = {"commands": [{"arm": "left", "kind": "cartesian_pose", "values": pose,
                                      "frame": "world"}], "request_stop": False}
            self.assertEqual(decode_action(json.dumps(document), self.config).commands[0].values, tuple(pose))
            pose_transform(pose)
        for scalar in (0.998, 1.002):
            pose = [0.3, 0.4, 0.2, scalar, 0., 0., 0.]
            document = {"commands": [{"arm": "left", "kind": "cartesian_pose", "values": pose,
                                      "frame": "world"}], "request_stop": False}
            with self.assertRaises(ValueError):
                decode_action(json.dumps(document), self.config)
            with self.assertRaises(ValueError):
                pose_transform(pose)

    def test_invalid_model_output_is_task_failure_not_service_error(self):
        settings = GPTSettings("https://example.org/openai", "gpt-6-astra")
        client = Mock()
        client.generate.return_value = ModelReply('{"unexpected":true}', settings.model, "fixture", {}, 0.)
        _, result, _, robot = self.run_case(GPTPolicy(self.config, settings, client=client))
        self.assertEqual(result["termination_reason"], "invalid_model_action")
        self.assertTrue(result["task_evaluated"])
        self.assertFalse(result["success"])
        self.assertEqual(result["decisions"], 1)
        self.assertFalse(robot.applied)

    def test_cleanup_error_cannot_preserve_a_passing_check_result(self):
        class BrokenClose(MockBackend):
            def close(self):
                raise RuntimeError("cleanup failed")
        with redirect_stdout(io.StringIO()):
            path, result = run_episode(self.config, BrokenClose(), MockRobot(), RunningTask(),
                                       NoopPolicy(self.config.control_dt_s), self.root,
                                       summarize=lambda r: {**r, "passed": True, "task_success": True})
        self.assertFalse(result["passed"])
        self.assertIsNone(result["success"])
        self.assertIsNone(result["task_success"])
        self.assertEqual(result["termination_reason"], "cleanup_error")
        self.assertEqual(json.loads((path / "result.json").read_text()), result)

    def test_execution_options_reject_impossible_budget(self):
        for data in ({"max_action_steps": 0}, {"max_action_steps": 2, "completion_samples": 3},
                     {"wait_steps": True}, {"max_decisions": 0}):
            with self.subTest(data=data), self.assertRaises(ValueError):
                ExecutionSettings.from_config(replace(self.config, raw={**self.config.raw, "execution": data}))


if __name__ == '__main__':
    unittest.main()
