"""Shared workflow regressions for the maintained L1 task pool."""

import unittest

from robot_tasks.config import ExecutionSettings
from robot_tasks.registry import resolve_task


class L1ProtocolTests(unittest.TestCase):
    def test_all_l1_tasks_use_shared_workflow_and_observation_arm(self):
        _, reference = resolve_task("GPT-PICK-PLACE")
        from robot_tasks.tasks.bench.task import BenchTask
        for name in ("L1-01", "L1-03", "L1-05", "L1-07", "L1-08", "L1-09"):
            spec, config = resolve_task(name)
            self.assertTrue(spec.implemented)
            self.assertEqual(config.robot.source.name, "r5_tuojing_lab_l1.toml")
            self.assertEqual(config.cameras, reference.cameras)
            manipulator = "right" if name in {"L1-01", "L1-03"} else "left"
            observer = "left" if manipulator == "right" else "right"
            self.assertEqual(config.arm_roles, {manipulator: "manipulation", observer: "observation"})
            for section in ("observation", "policy", "recording"):
                self.assertEqual(config.raw[section], reference.raw[section], (name, section))
            execution = dict(config.raw["execution"])
            self.assertEqual(execution, reference.raw["execution"])
            self.assertNotIn("policy_options", config.raw)
            self.assertEqual(config.observation_arms, (observer,))
            self.assertFalse(config.include_depth)
            self.assertEqual(ExecutionSettings.from_config(config).max_decisions, 100)
            self.assertNotIn("reasoning_effort", config.raw["policy"])
            self.assertEqual(config.control_dt_s, reference.control_dt_s)
            self.assertEqual(config.physics_dt_s, reference.physics_dt_s)
            prompt = BenchTask.prepare_config(config).instruction
            self.assertNotIn("metadata", prompt)
            self.assertNotIn("sha256", prompt)
            self.assertIn("observation arm", prompt)
            self.assertNotIn("keep the other arm still", prompt)
