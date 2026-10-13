"""Initial-layout selection must propagate beyond the model's prose."""

from dataclasses import replace
import unittest
from copy import deepcopy
from unittest.mock import patch

from robot_tasks.registry import resolve_task
from robot_tasks.robot_roles import validate_role_command
from robot_tasks.robots.kinematics import R5Kinematics
from robot_tasks.tasks.bench.arm_selection import resolve_bench_spec
from robot_tasks.tasks.bench.spec import build_spec


class ArmSelectionTests(unittest.TestCase):
    def setUp(self):
        self.binding, config = resolve_task("L1-01")
        raw = deepcopy(config.raw)
        raw["task"].update(arm_selection="nearest_initial_object", manipulation_object="marker")
        raw["robot"].update(active_arms=["left"], observation_arms=["right"])
        self.config = replace(config, raw=raw, arms=("left",), observation_arms=("right",))

    def test_current_layout_selects_left_and_updates_model_roles(self):
        config = self.config
        spec = resolve_bench_spec(config.raw["task"], config.robot)
        self.assertEqual(config.arms, ("left",))
        self.assertEqual(config.observation_arms, ("right",))
        self.assertEqual(config.raw["robot"]["active_arms"], ["left"])
        self.assertEqual(spec.arms, config.arms)
        self.assertEqual(spec.observers, config.observation_arms)
        audit = spec.parameters["arm_selection"]
        self.assertAlmostEqual(audit["distances_m"]["left"], .16042, places=4)
        self.assertAlmostEqual(audit["distances_m"]["right"], .39108, places=4)
        self.assertTrue(audit["roles_fixed_for_episode"])
        prepared = self.binding.factory.prepare_config(config)
        self.assertIn("use the LEFT arm for manipulation", prepared.instruction)
        self.assertIn("RIGHT arm for observation only", prepared.instruction)
        self.assertNotIn(str(audit["distances_m"]["right"]), prepared.instruction)

    def test_moving_initial_marker_near_right_changes_config_and_prompt(self):
        spec = build_spec("L1-01")
        tcp = R5Kinematics(self.config.robot).forward((0.,) * 6, "right")
        marker = spec.design.articles[0]
        moved = replace(marker, initial=(*tcp[:3], *marker.initial[3:]))
        spec = replace(spec, design=replace(spec.design, articles=(moved,)))
        with patch("robot_tasks.tasks.bench.arm_selection.build_spec", return_value=spec):
            spec = resolve_bench_spec(self.config.raw["task"], self.config.robot)
            config = replace(self.config, arms=spec.arms, observation_arms=spec.observers)
            prepared = self.binding.factory.prepare_config(config)
        self.assertEqual(config.arms, ("right",))
        self.assertEqual(config.observation_arms, ("left",))
        self.assertIn("use the RIGHT arm for manipulation", prepared.instruction)
        self.assertIn("LEFT arm for observation only", prepared.instruction)

    def test_equal_distance_is_deterministic_even_if_mount_order_changes(self):
        robot = replace(self.config.robot, mounts=tuple(reversed(self.config.robot.mounts)))
        with patch.object(R5Kinematics, "forward", return_value=(0., 0., 1., 1., 0., 0., 0.)):
            spec = resolve_bench_spec(self.config.raw["task"], robot)
        self.assertEqual(spec.arms, ("left",))
        self.assertEqual(spec.observers, ("right",))

    def test_only_selected_arm_can_receive_gripper_commands(self):
        validate_role_command(self.config, "left", "gripper")
        validate_role_command(self.config, "right", "cartesian")
        with self.assertRaisesRegex(ValueError, "Observation arm right"):
            validate_role_command(self.config, "right", "gripper")

    def test_invalid_initial_geometry_cannot_silently_choose_an_arm(self):
        with patch.object(R5Kinematics, "forward", return_value=(float("nan"), 0., 1.)):
            with self.assertRaisesRegex(ValueError, "Nonfinite"):
                resolve_bench_spec(self.config.raw["task"], self.config.robot)

    def test_invalid_mode_target_and_non_l1_are_rejected(self):
        for update, message in (({"arm_selection": "typo"}, "Unknown"),
                                ({"manipulation_object": "missing"}, "movable task article"),
                                ({"id": "L2-01"}, "single-arm L1")):
            with self.subTest(update=update), self.assertRaisesRegex(ValueError, message):
                resolve_bench_spec({**self.config.raw["task"], **update}, self.config.robot)

    def test_fixed_tasks_keep_existing_roles(self):
        for task_id in ("L1-05",):
            with self.subTest(task=task_id):
                binding, config = resolve_task(task_id)
                prepared = binding.factory.prepare_config(config)
                self.assertEqual(prepared.arms, ("left",))
                self.assertEqual(prepared.observation_arms, ("right",))

    def test_charger_real_prompt_overrides_previous_nearest_arm(self):
        binding, config = resolve_task("L1-03")
        prepared = binding.factory.prepare_config(config)
        self.assertEqual(config.raw["task"]["arm_selection"], "real_prompt")
        self.assertEqual(prepared.arms, ("left",))
        self.assertEqual(prepared.observation_arms, ("right",))
        self.assertEqual(prepared.instruction, config.instruction)


if __name__ == "__main__":
    unittest.main()
