"""Physical task completion must not bypass measured return-home, offline."""
from dataclasses import asdict, replace
import json
import math
import unittest

from robot_tasks.registry import resolve_task
from robot_tasks.tasks.completion import completion_instruction, require_home
from robot_tasks.types import ArmState, Assessment

IDS = ("L2-03", "L2-04", "L2-06", "L2-07", "L3-05", "L3-14", "L3-15")


class CompletionTests(unittest.TestCase):
    def test_immediate_goal_tasks_do_not_wait_for_release_or_home(self):
        for task_id in ("L2-01", "L3-03", "L3-11"):
            with self.subTest(task=task_id):
                entry, config = resolve_task(task_id)
                prepared = entry.factory.prepare_config(config)
                self.assertFalse(prepared.raw["completion"]["return_home"])
                self.assertNotIn("Final completion protocol:", prepared.instruction)
                self.assertEqual(completion_instruction(prepared), "")
                done = Assessment("succeeded", "fully detached", {})
                self.assertIs(require_home(done, {}, prepared), done)

    def setUp(self):
        _, self.config = resolve_task("L2-06")
        self.home = {m.name: ArmState(joint_positions_rad=m.home_positions_rad)
                     for m in self.config.robot.mounts}
        self.done = Assessment("succeeded", "physical goal and release", {"goal_complete": True})

    def test_nonverbatim_tasks_instruct_release_and_measured_home(self):
        from types import SimpleNamespace
        from robot_tasks.policies.gpt import GPTPolicy

        for task_id in IDS:
            with self.subTest(task=task_id):
                entry, config = resolve_task(task_id)
                config = replace(config, raw={**config.raw, "task": {
                    **config.raw["task"], "verbatim_instruction": False}})
                prepared = entry.factory.prepare_config(config)
                prompt = prepared.instruction
                self.assertTrue(config.raw["completion"]["return_home"])
                self.assertIn("RELEASE", prompt)
                self.assertIn("BOTH arms back", prompt)
                self.assertIn("measured joint state", prompt)
                self.assertEqual(prompt.count("Final completion protocol:"), 1)
                policy = GPTPolicy(prepared, SimpleNamespace(), client=object())
                self.assertIn("joint_positions_rad",
                              policy.schema["properties"]["commands"]["items"]["required"])

    def test_model_can_command_both_home_targets_through_existing_joint_planner_channel(self):
        from robot_tasks.policies.sparse_action import FIELDS, decode_sparse_action, sparse_schema
        schema = sparse_schema(self.config.command_arms, joint_targets=True)
        self.assertIn("joint_positions_rad", schema["properties"]["commands"]["items"]["required"])
        rows = [{"arm": arm, **dict.fromkeys(FIELDS),
                 "joint_positions_rad": list(state.joint_positions_rad)}
                for arm, state in self.home.items()]
        action = decode_sparse_action(json.dumps({"commands": rows, "request_stop": False}),
                                      self.config, None)
        self.assertEqual(len(action.commands), 2)
        self.assertEqual({c.arm for c in action.commands}, {"left", "right"})
        self.assertTrue(all(c.kind == "joint_position" and c.values == (0.,) * 6
                            for c in action.commands))

    def test_home_interface_rejects_ambiguous_invalid_or_unavailable_joint_targets(self):
        from robot_tasks.policies.sparse_action import FIELDS, decode_sparse_action
        from robot_tasks.types import InvalidPolicyAction
        row = {"arm": "left", **dict.fromkeys(FIELDS), "joint_positions_rad": [0.] * 6}
        for patch in ({"x_m": .2}, {"joint_positions_rad": [0.] * 5},
                      {"joint_positions_rad": [math.nan] * 6},
                      {"joint_positions_rad": [True] * 6},
                      {"joint_positions_rad": [100.] * 6}):
            with self.subTest(patch=patch), self.assertRaises(InvalidPolicyAction):
                decode_sparse_action(json.dumps({"commands": [{**row, **patch}], "request_stop": False}),
                                     self.config, None)
        _, original = resolve_task("L1-01")
        with self.assertRaises(InvalidPolicyAction):
            decode_sparse_action(json.dumps({"commands": [row], "request_stop": False}), original, None)

    def test_only_complete_physical_goal_plus_both_home_succeeds(self):
        result = require_home(self.done, self.home, self.config)
        self.assertEqual(result.status, "succeeded")
        self.assertTrue(result.metrics["completion"]["both_arms_home"])
        running = Assessment("running", "still holding orange", {"goal_complete": False})
        self.assertEqual(require_home(running, self.home, self.config).status, "running")
        for name in self.home:
            with self.subTest(arm=name):
                states = dict(self.home)
                q = list(states[name].joint_positions_rad)
                q[0] += .10
                states[name] = replace(states[name], joint_positions_rad=tuple(q))
                result = require_home(self.done, states, self.config)
                self.assertEqual(result.status, "running")
                self.assertEqual(result.metrics["stage"], "return_home")
                self.assertFalse(result.metrics["goal_complete"])
                self.assertTrue(result.metrics["completion"]["task_and_release_complete"])
                json.dumps(asdict(result), allow_nan=False)

    def test_missing_or_nonfinite_readback_cannot_pass_and_stays_serializable(self):
        for states in ({}, {**self.home, "right": ArmState()},
                       {**self.home, "left": ArmState(joint_positions_rad=(math.nan,) * 6)}):
            result = require_home(self.done, states, self.config)
            self.assertEqual(result.status, "running")
            json.dumps(asdict(result), allow_nan=False)

    def test_home_tolerance_is_small_finite_and_not_exact_float_equality(self):
        states = {k: replace(v, joint_positions_rad=tuple(q + .01 for q in v.joint_positions_rad))
                  for k, v in self.home.items()}
        self.assertEqual(require_home(self.done, states, self.config).status, "succeeded")
        for tolerance in (0, -1, math.nan, math.inf):
            config = replace(self.config, raw={**self.config.raw, "completion": {
                "return_home": True, "home_tolerance_rad": tolerance}})
            with self.assertRaises(ValueError):
                require_home(self.done, states, config)

    def test_failures_and_unmodified_tasks_are_not_reinterpreted(self):
        failed = Assessment("failed", "lost object", {})
        self.assertIs(require_home(failed, self.home, self.config), failed)
        _, config = resolve_task("L1-01")
        self.assertEqual(completion_instruction(config), "")
        self.assertIs(require_home(self.done, {}, config), self.done)
