"""Offline reset protocol tests. No simulator, physical script or model service."""

from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from robot_tasks.backends.mock import MockBackend, MockRobot, MockTask
from robot_tasks.config import ExecutionSettings, load_config
from robot_tasks.policies.gpt import GPTPolicy
from robot_tasks.policies.model_config import GPTSettings
from robot_tasks.policies.responses import ModelError, ModelReply
from robot_tasks.runner import run_episode
from robot_tasks.types import ArmState, Assessment, Feedback, RGBFrame


ROOT = Path(__file__).resolve().parents[1]


class ResetBackend(MockBackend):
    def open(self, config):
        super().open(config)
        self.resets = 0

    def reset(self, seed):
        super().reset(seed)
        self.resets += 1

    def capture_rgb(self):
        # Distinct offline fixtures prove that post-reset images are re-acquired.
        value = (self.resets + int(round(self.t * 120))) % 256
        return tuple(RGBFrame(name, self.t, 2, 2, bytes([value] * 12)) for name in self.config.cameras)


class ResetRobot(MockRobot):
    def initialize(self, backend, config):
        super().initialize(backend, config)
        self.resets = 0
        self.applies = 0
        self.pending = False

    def reset(self, seed):
        self.resets += 1
        self.pending = False

    def apply(self, action):
        self.applies += 1
        self.pending = bool(action.commands)

    def state(self):
        return {arm: ArmState(gripper_width_m=.04 if self.pending else .07) for arm in self.arms}

    def feedback(self):
        return Feedback("executing" if self.pending else "completed")


class AttemptTask(MockTask):
    def __init__(self, *, always_fail=False, retryable=True, fail_reset=False):
        self.resets = 0
        self.always_fail, self.retryable, self.fail_reset = always_fail, retryable, fail_reset

    def build_scene(self, backend, config):
        self.settled_time = ExecutionSettings.from_config(config).settle_steps * config.control_dt_s

    def reset(self, backend, seed):
        self.resets += 1
        if self.fail_reset and self.resets > 1:
            raise RuntimeError("Reset fixture failed")

    def evaluate(self, state, feedback):
        if state.values["simulation_time_s"] <= self.settled_time + 1e-8:
            return Assessment("running", "Fresh attempt", {"lift_confirmed": False})
        if self.resets == 1 or self.always_fail:
            return Assessment("failed", "Offline fixture requested a retry", {"hidden_target": [123., 456.]},
                              retryable=self.retryable)
        return Assessment("succeeded", "Offline fixture completed", {"lift_confirmed": True})


class RetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        base = load_config(ROOT / "configs/tasks/gpt_single_arm_pick_place.toml")
        self.config = replace(base, include_depth=False, save_depth=False, raw={**base.raw, "execution": {
            "max_action_steps": 4, "max_decisions": 100, "retry_failed_attempts": True},
            "policy_options": {"retain_text_history": True}})
        self.settings = GPTSettings("https://example.org/openai", "fixture-model", max_calls=100)
        self.client = Mock()
        self.client.generate.return_value = self.reply(json.dumps({"request_stop": False, "commands": [
            {"arm": "left", "kind": "gripper", "values": [.04], "frame": "world"}]}))

    def reply(self, text):
        return ModelReply(text, self.settings.model, "offline-fixture", {}, 0.)

    def run_case(self, *, config=None, task=None, backend=None, record_video=False):
        config = config or self.config
        self.backend, self.robot = backend or ResetBackend(), ResetRobot()
        self.task = task or AttemptTask()
        self.policy = GPTPolicy(config, self.settings, client=self.client)
        self.policy.reset = Mock(wraps=self.policy.reset)
        with redirect_stdout(io.StringIO()):
            self.path, result = run_episode(config, self.backend, self.robot, self.task, self.policy,
                                            self.root, record_video=record_video)
        self.events = [json.loads(line) for line in (self.path / "episode.jsonl").read_text().splitlines()]
        return result

    def test_reset_keeps_history_budget_and_failure_evidence_but_refreshes_observation(self):
        result = self.run_case()
        self.assertTrue(result["success"])
        self.assertFalse(result["first_attempt_success"])
        self.assertEqual(result["success_attempt"], 2)
        self.assertEqual(result["model_calls"], 2)
        self.assertEqual(result["reset_count"], 1)
        self.assertEqual(result["failed_attempts"], 1)
        self.assertEqual(result["action_timeouts"], 0)
        self.assertEqual(self.robot.resets, 2)
        self.assertEqual(self.backend.resets, 2)
        self.policy.reset.assert_called_once()
        calls = self.client.generate.call_args_list
        messages = calls[1].kwargs["messages"]
        self.assertEqual(len(messages), 3)  # Previous observation/action plus fresh input.
        fresh = json.loads(messages[-1]["content"][0]["text"])
        self.assertEqual(fresh["step"], 2)
        self.assertEqual(fresh["simulation_time_s"], 0.)
        self.assertEqual(fresh["robot"]["left"]["gripper_width_m"], .07)
        self.assertEqual(fresh["feedback"]["status"], "attempt_reset")
        self.assertEqual(fresh["feedback"]["metrics"]["remaining_decisions"], 99)
        self.assertNotIn("hidden_target", json.dumps(messages))
        self.assertNotEqual(messages[-1]["content"][-1], calls[0].kwargs["messages"][-1]["content"][-1])
        failed = next(e for e in self.events if e["event"] == "transition")
        reset = next(e for e in self.events if e["event"] == "attempt_reset")
        before = self.path / failed["observation"]["rgb"][0]["path"]
        after = self.path / reset["observation"]["rgb"][0]["path"]
        self.assertNotEqual(before, after)
        self.assertNotEqual(before.read_bytes(), after.read_bytes())
        self.assertEqual([a["status"] for a in result["attempts"]], ["failed", "succeeded"])
        self.assertEqual([a["decision_start"] for a in result["attempts"]], [1, 2])
        self.assertAlmostEqual(result["simulation_time_s"], 2 * self.config.control_dt_s)
        self.assertAlmostEqual(result["final_attempt_time_s"], self.config.control_dt_s)

    def test_decision_note_survives_scene_reset_without_a_fresh_call_budget(self):
        current = replace(self.config, raw={**self.config.raw,
            "policy_options": {"retain_text_history": True, "decision_notes": True}})
        doc = json.loads(self.client.generate.return_value.text)
        doc["note"] = "The object is visible; close the left gripper."
        self.client.generate.return_value = self.reply(json.dumps(doc))
        result = self.run_case(config=current)
        self.assertEqual(result["model_calls"], 2)
        self.assertEqual(result["reset_count"], 1)
        self.policy.reset.assert_called_once()
        second_messages = self.client.generate.call_args_list[1].kwargs["messages"]
        self.assertIn(doc["note"], second_messages[1]["content"])
        self.assertIn("attempt_reset", second_messages[-1]["content"][0]["text"])
        notes = [e["action"]["note"] for e in self.events if e["event"] == "decision"]
        self.assertEqual(notes, [doc["note"], doc["note"]])

    def test_100_calls_are_shared_and_last_failure_does_not_start_an_unused_attempt(self):
        result = self.run_case(task=AttemptTask(always_fail=True))
        self.assertEqual(self.client.generate.call_count, 100)
        self.assertEqual(result["model_calls"], 100)
        self.assertEqual(result["attempt_count"], 100)
        self.assertEqual(result["failed_attempts"], 100)
        self.assertEqual(result["reset_count"], 99)
        self.assertEqual(result["termination_reason"], "model_call_limit")
        self.assertFalse(result["success"])
        self.policy.reset.assert_called_once()

    def test_invalid_response_after_reset_uses_shared_budget_and_does_not_reset_again(self):
        valid = self.client.generate.return_value
        self.client.generate.side_effect = [valid, self.reply("not-json"), valid]
        result = self.run_case()
        self.assertTrue(result["success"])
        self.assertEqual(result["model_calls"], 3)
        self.assertEqual(result["invalid_model_actions"], 1)
        self.assertEqual(result["reset_count"], 1)
        self.assertEqual(result["attempts"][1]["decisions"], 2)
        third = json.loads(self.client.generate.call_args_list[2].kwargs["messages"][-1]["content"][0]["text"])
        self.assertIn("Last reset notice", third["feedback"]["message"])
        self.assertIn("previous poses are no longer current", third["feedback"]["message"])

    def test_disabled_protocol_and_nonretryable_failures_do_not_reset(self):
        for enabled, retryable in ((False, True), (True, False)):
            with self.subTest(enabled=enabled, retryable=retryable):
                config = replace(self.config, raw={**self.config.raw, "execution": {
                    **self.config.raw["execution"], "retry_failed_attempts": enabled}})
                result = self.run_case(config=config, task=AttemptTask(retryable=retryable))
                self.assertFalse(result["success"])
                self.assertEqual(result["model_calls"], 1)
                self.assertEqual(self.robot.resets, 1)

    def test_settling_and_all_attempts_share_physics_budget_and_recording(self):
        class Video:
            def __init__(self, directory, settings):
                self.count = 0
            def add(self, frame):
                self.count += 1
            def close(self):
                pass
        config = replace(self.config, max_steps=4, raw={**self.config.raw, "execution": {
            "max_action_steps": 1, "settle_steps": 1, "retry_failed_attempts": True}})
        with patch("robot_tasks.video.MotionVideo", Video):
            result = self.run_case(config=config, task=AttemptTask(always_fail=True), record_video=True)
        self.assertEqual(result["model_calls"], 2)
        self.assertEqual(result["reset_count"], 1)
        self.assertEqual(result["termination_reason"], "max_steps")
        self.assertEqual(result["completed_steps"], 4)
        self.assertTrue(all(v["frames"] == 4 for v in result["videos"].values()))
        self.assertEqual([e["tick"] for e in self.events if e["event"] == "physics"], [1, 2, 3, 4])

    def test_wall_clock_budget_is_not_restarted(self):
        clock = [0.]
        class TimedBackend(ResetBackend):
            def step(self, dt):
                super().step(dt)
                clock[0] += 1.
        config = replace(self.config, max_wall_time_s=1.5)
        with patch("robot_tasks.runner.time.monotonic", side_effect=lambda: clock[0]):
            result = self.run_case(config=config, backend=TimedBackend(), task=AttemptTask(always_fail=True))
        self.assertEqual(result["termination_reason"], "wall_time_limit")
        self.assertEqual(result["model_calls"], 2)
        self.assertEqual(result["reset_count"], 1)

    def test_reset_failure_is_an_error_and_previous_failure_is_preserved(self):
        result = self.run_case(task=AttemptTask(fail_reset=True))
        self.assertEqual(result["status"], "error")
        self.assertIsNone(result["success"])
        self.assertEqual(result["failed_attempts"], 1)
        self.assertEqual(result["reset_count"], 0)
        self.assertIn("Reset fixture failed", result["error"])
        self.assertEqual(self.client.generate.call_count, 1)

    def test_provider_error_after_reset_is_not_a_task_success(self):
        self.client.generate.side_effect = [self.client.generate.return_value, ModelError("Service fixture failed")]
        result = self.run_case()
        self.assertEqual(result["status"], "error")
        self.assertIsNone(result["success"])
        self.assertEqual([a["status"] for a in result["attempts"]], ["failed", "not_evaluated"])

    def test_failure_during_stop_observation_retries_and_does_not_keep_stop_flag(self):
        self.client.generate.return_value = self.reply('{"commands":[],"request_stop":true}')
        config = replace(self.config, raw={**self.config.raw, "execution": {
            "max_decisions": 3, "stop_steps": 1, "retry_failed_attempts": True}})
        result = self.run_case(config=config, task=AttemptTask(always_fail=True))
        self.assertEqual(result["model_calls"], 3)
        self.assertEqual(result["reset_count"], 2)
        self.assertEqual(result["failed_attempts"], 3)

    def test_reset_requires_explicit_boolean_and_visible_feedback(self):
        for config in (replace(self.config, include_feedback=False),
                       replace(self.config, raw={**self.config.raw, "execution": {"retry_failed_attempts": 1}})):
            with self.assertRaises(ValueError):
                ExecutionSettings.from_config(config)


if __name__ == "__main__":
    unittest.main()
