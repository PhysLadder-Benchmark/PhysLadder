"""Fixed generation retries reuse observations; no paid/network calls."""

import json
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from io import StringIO
from pathlib import Path
from unittest.mock import Mock, patch

from fixtures import RGBDBackend, decision_document, gripper_command, robot_config

from robot_tasks.backends.mock import MockRobot, MockTask
from robot_tasks.policies.gpt import GPTPolicy
from robot_tasks.policies.model_config import GPTSettings
from robot_tasks.policies.responses import ModelError, ModelReply, RetryableModelError
from robot_tasks.runner import run_episode
from robot_tasks.types import Assessment, Feedback


def failed():
    return RetryableModelError("Provider reported failed/incomplete generation")


def reply(*, move=False):
    doc = decision_document(commands=[gripper_command()] if move else [])
    return ModelReply(json.dumps(doc), "fixture", "offline", {}, 0.0)


class ModelRetryTests(unittest.TestCase):
    def run_fixture(self, replies, calls=10, *, interrupt=False):
        config = robot_config()
        config = replace(
            config,
            max_steps=0,
            raw={
                **config.raw,
                "execution": {
                    "max_decisions": calls,
                    "max_action_steps": 1,
                    "completion_samples": 1,
                },
            },
        )
        client = Mock()
        client.generate.side_effect = replies
        policy = GPTPolicy(
            config,
            GPTSettings("https://example.invalid", "fixture", max_calls=calls),
            client=client,
        )
        robot, task = MockRobot(), MockTask()
        robot.apply = Mock()
        robot.feedback = Mock(return_value=Feedback("completed"))
        task.evaluate = Mock(return_value=Assessment("running", "Offline fixture"))
        with (
            tempfile.TemporaryDirectory() as directory,
            redirect_stdout(StringIO()),
            patch(
                "robot_tasks.runner.time.sleep",
                side_effect=KeyboardInterrupt if interrupt else None,
            ) as sleep,
        ):
            output, result = run_episode(
                config, RGBDBackend(), robot, task, policy, Path(directory)
            )
            events = [
                json.loads(l)
                for l in (output / "episode.jsonl").read_text().splitlines()
            ]
        return result, events, policy, robot, client, sleep

    def test_three_retries_then_success_execute_once_and_count_four_calls(self):
        result, events, policy, robot, client, sleep = self.run_fixture(
            [failed(), failed(), failed(), reply(move=True)], calls=4
        )
        self.assertEqual(result["status"], "finished")
        self.assertEqual(result["model_calls"], 4)
        self.assertEqual(client.generate.call_count, 4)
        self.assertEqual(result["model_retry_attempts"], 3)
        self.assertEqual(result["generation_failures"], 3)
        robot.apply.assert_called_once()
        self.assertEqual(len(policy.history), 1)
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [2.0, 4.0, 8.0])
        retries = [e for e in events if e["event"] == "model_retry"]
        self.assertEqual([e["failure_number"] for e in retries], [1, 2, 3])
        self.assertTrue(all(e["simulation_time_s"] == 0.0 for e in retries))
        exchanges = [e["exchange"] for e in events if e["event"] == "model_exchange"]
        self.assertTrue(all(e["frames"] == exchanges[0]["frames"] for e in exchanges))
        self.assertEqual([e["retryable"] for e in exchanges[:3]], [True] * 3)

    def test_four_consecutive_failures_stop_without_fifth_call_or_partial_action(self):
        result, events, policy, robot, client, _sleep = self.run_fixture([failed()] * 4)
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["model_calls"], 4)
        self.assertEqual(result["model_retry_attempts"], 3)
        self.assertEqual(client.generate.call_count, 4)
        self.assertEqual(result["completed_steps"], 0)
        self.assertFalse(result["task_evaluated"])
        self.assertIsNone(result["success"])
        robot.apply.assert_not_called()
        self.assertFalse(policy.history)
        exhausted = [e for e in events if e["event"] == "model_retry_exhausted"]
        self.assertEqual(exhausted[0]["retry_exhaustion"], "retry_limit")

    def test_success_resets_retry_streak_and_retains_accepted_history(self):
        result, events, policy, robot, client, sleep = self.run_fixture(
            [failed(), failed(), reply(), failed(), failed(), reply()], calls=6
        )
        self.assertEqual(result["status"], "finished")
        self.assertEqual(result["model_retry_attempts"], 4)
        self.assertEqual(client.generate.call_count, 6)
        self.assertEqual(robot.apply.call_count, 2)
        self.assertEqual(len(policy.history), 2)
        exchanges = [e["exchange"] for e in events if e["event"] == "model_exchange"]
        self.assertEqual([e["history_steps"] for e in exchanges], [0, 0, 0, 1, 1, 1])
        self.assertEqual(
            [c.args[0] for c in sleep.call_args_list], [2.0, 4.0, 2.0, 4.0]
        )

    def test_retry_cannot_exceed_100_total_requests(self):
        result, events, _policy, _robot, client, _sleep = self.run_fixture(
            [reply()] * 98 + [failed(), failed()], calls=100
        )
        self.assertEqual(result["model_calls"], 100)
        self.assertEqual(client.generate.call_count, 100)
        self.assertEqual(result["model_retry_attempts"], 1)
        self.assertEqual(result["status"], "error")
        self.assertFalse(result["task_evaluated"])
        exhausted = [e for e in events if e["event"] == "model_retry_exhausted"]
        self.assertEqual(exhausted[0]["retry_exhaustion"], "model_call_limit")

    def test_permanent_error_is_not_retried(self):
        result, _events, _policy, robot, _client, sleep = self.run_fixture(
            [ModelError("Responses HTTP 401")]
        )
        self.assertEqual(result["model_calls"], 1)
        self.assertEqual(result["model_retry_attempts"], 0)
        sleep.assert_not_called()
        robot.apply.assert_not_called()

    def test_interrupt_during_backoff_does_not_start_another_call(self):
        result, _events, _policy, robot, client, _sleep = self.run_fixture(
            [failed()], interrupt=True
        )
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(client.generate.call_count, 1)
        self.assertEqual(result["model_calls"], 1)
        self.assertEqual(result["model_retry_attempts"], 0)
        robot.apply.assert_not_called()


if __name__ == "__main__":
    unittest.main()
