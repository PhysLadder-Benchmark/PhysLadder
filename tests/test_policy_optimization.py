import base64
from dataclasses import replace
from io import BytesIO
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

from robot_tasks.cameras.geometry import project_world, resized_intrinsics
from robot_tasks.config import ExecutionSettings, load_config
from robot_tasks.policies.gpt import GPTPolicy, prepare_frame
from robot_tasks.policies.model_config import GPTSettings
from robot_tasks.policies.options import PolicyOptions
from robot_tasks.policies.responses import ModelReply
from robot_tasks.policies.sparse_action import FIELDS, REFERENCE, decode_sparse_action, orientation_degrees
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.robots.control import width_to_joints
from robot_tasks.robots.transforms import origin_transform, pose_errors, transform_pose
from robot_tasks.types import Action, ArmCommand, ArmState, Feedback, Observation, PolicyInput, RGBFrame


ROOT = Path(__file__).resolve().parents[1]


class OptimizationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "configs/tasks/gpt_single_arm_pick_place_optimized.toml")
        t = origin_transform((.3, .4, .2), (0., 0., 0.))
        t[:3, :3] = REFERENCE
        self.pose = transform_pose(t)
        self.frame = RGBFrame("overview", 1., 100, 80, bytes((20, 40, 60)) * 8000,
                              {"intrinsics_fx_fy_cx_cy": [80, 80, 50, 40], "resolution": [100, 80],
                               "distortion": [0.] * 5, "T_world_camera_pose_wxyz": [0, 0, 1, 1, 0, 0, 0]})
        self.obs = Observation(1., (self.frame,), {"left": ArmState(tcp_pose_world_wxyz=self.pose)}, Feedback("completed"))
        self.settings = GPTSettings("https://example.invalid", "test", max_calls=8)
        self.client = Mock()
        self.client.generate.return_value = ModelReply('{"commands":[],"request_stop":false}', "test", "id", {}, 0.)

    def sparse(self, **values):
        row = {"arm": "left", **dict.fromkeys(FIELDS), **values}
        return decode_sparse_action(json.dumps({"commands": [row], "request_stop": False}), self.config, self.obs)

    def test_sparse_position_holds_measured_axes_and_quaternion(self):
        command, = self.sparse(z_m=.25).commands
        self.assertEqual(command.values, (*self.pose[:2], .25, *self.pose[3:]))

    def test_sparse_angles_rotate_about_tcp_without_translating_it(self):
        command, = self.sparse(yaw_deg=45).commands
        self.assertEqual(command.values[:3], self.pose[:3])
        np.testing.assert_allclose(orientation_degrees(command.values), (0, 0, 45), atol=1e-8)

    def test_gripper_only_does_not_create_motion(self):
        command, = self.sparse(gripper_width_m=.06).commands
        self.assertEqual(command.kind, "gripper")

    def test_sparse_invalid_values_and_unknown_arm_are_rejected(self):
        for values in ({"arm": "right", "z_m": .2}, {"z_m": True}, {"z_m": float("nan")},
                       {"roll_deg": 500}, {"gripper_width_m": .1}, {}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.sparse(**values)

    def test_partial_motion_requires_measured_tcp(self):
        self.obs = replace(self.obs, robot={})
        with self.assertRaisesRegex(ValueError, "Measured TCP"):
            self.sparse(z_m=.2)

    def test_text_history_survives_image_eviction_and_episode_reset(self):
        policy = GPTPolicy(self.config, self.settings, client=self.client)
        for i in range(4):
            policy.act(PolicyInput(f"instruction_{i}", self.obs, i + 1))
        self.assertEqual(len(policy.history), 4)
        messages = self.client.generate.call_args.kwargs["messages"]
        self.assertIn("instruction_0", json.dumps(messages))
        self.assertFalse(any(item["type"] == "input_image" for item in policy.history[0][0]["content"]))
        self.assertEqual(sum(item["type"] == "input_image" for user, _ in policy.history for item in user["content"]), 2)
        policy.reset(12)
        policy.act(PolicyInput("fresh_episode", self.obs, 1))
        self.assertEqual(len(self.client.generate.call_args.kwargs["messages"]), 1)
        self.assertNotIn("instruction_0", json.dumps(self.client.generate.call_args.kwargs["messages"]))

    def test_baseline_does_not_disclose_calibration_or_overlay(self):
        policy = GPTPolicy(self.config, self.settings, client=self.client)
        policy.act(PolicyInput("task", self.obs, 1))
        messages = self.client.generate.call_args.kwargs["messages"]
        self.assertNotIn("T_world_camera", json.dumps(messages))
        image = Image.open(BytesIO(base64.b64decode(messages[-1]["content"][-1]["image_url"].split(",")[1])))
        self.assertEqual(image.getpixel((0, 0)), (20, 40, 60))
        self.assertNotIn("source_x_m", policy.instructions)
        self.assertNotIn("transfer_y_m", policy.instructions)

    def test_disclosed_calibration_tracks_actual_encoded_size(self):
        config = replace(self.config, raw={**self.config.raw, "policy_options": {"camera_calibration": True}})
        policy = GPTPolicy(config, replace(self.settings, image_max_edge=64), client=self.client)
        policy.act(PolicyInput("task", self.obs, 1))
        content = self.client.generate.call_args.kwargs["messages"][-1]["content"]
        calibration = json.loads(content[1]["text"])["calibration"]
        self.assertEqual(calibration["resolution"], [64, 51])
        self.assertEqual(calibration["intrinsics_fx_fy_cx_cy"], list(resized_intrinsics((80,80,50,40), (100,80), (64,51))))

    def test_observation_timestamp_mismatch_rejected_before_request(self):
        policy = GPTPolicy(self.config, self.settings, client=self.client)
        with self.assertRaisesRegex(ValueError, "timestamps"):
            policy.act(PolicyInput("task", replace(self.obs, time_s=2.), 1))
        self.client.generate.assert_not_called()

    def test_calibration_requested_without_metadata_fails_before_request(self):
        config = replace(self.config, raw={**self.config.raw, "policy_options": {"camera_calibration": True}})
        policy = GPTPolicy(config, self.settings, client=self.client)
        with self.assertRaisesRegex(ValueError, "no camera metadata"):
            policy.act(PolicyInput("task", replace(self.obs, rgb=(replace(self.frame, calibration=None),)), 1))
        self.client.generate.assert_not_called()

    def test_visual_aids_preserve_raw_rgb(self):
        original = self.frame.data
        encoded, _, _ = prepare_frame(self.frame, 100, visual_aids=True, robot=self.obs.robot)
        self.assertEqual(self.frame.data, original)
        self.assertNotEqual(Image.open(BytesIO(encoded)).tobytes(), original)

    def test_visual_aids_reach_model_without_disclosing_camera_parameters(self):
        config = replace(self.config, raw={**self.config.raw, "policy_options": {
            **self.config.raw["policy_options"], "visual_aids": True, "camera_calibration": False}})
        policy = GPTPolicy(config, self.settings, client=self.client)
        original = self.frame.data
        policy.act(PolicyInput("task", self.obs, 1))
        content = self.client.generate.call_args.kwargs["messages"][-1]["content"]
        label = json.loads(content[1]["text"])
        self.assertNotIn("calibration", label)
        encoded = base64.b64decode(content[2]["image_url"].split(",", 1)[1])
        image = Image.open(BytesIO(encoded))
        self.assertNotEqual(image.tobytes(), original)
        self.assertEqual(self.frame.data, original)
        self.assertTrue(policy.last_exchange["frames"][0]["visual_aids"])

    def test_option_typos_and_invalid_timing_are_rejected(self):
        for options in ({"camera_calibration": "true"}, {"action_interface": "magic"}, {"unknown": True}):
            with self.assertRaises(ValueError):
                PolicyOptions.from_config(replace(self.config, raw={**self.config.raw, "policy_options": options}))
        with self.assertRaises(ValueError):
            ExecutionSettings.from_config(replace(self.config, raw={**self.config.raw, "execution": {"gripper_timing": "unknown"}}))

    def test_camera_axis_conversion_and_points_behind_camera(self):
        result = project_world([(0,0,-1), (.2,.1,-1), (0,0,1)], (0,0,0,1,0,0,0), (100,100,50,40))
        np.testing.assert_allclose(result[:2], [[50,40], [70,30]])
        self.assertTrue(np.isnan(result[2]).all())

    def test_resized_projection_preserves_pixel_centres(self):
        point = [[.12,-.08,-1]]
        original = project_world(point, (0,0,0,1,0,0,0), (80,80,50,40))[0]
        resized = project_world(point, (0,0,0,1,0,0,0), resized_intrinsics((80,80,50,40), (100,80), (50,40)))[0]
        np.testing.assert_allclose(resized, (original + .5) / 2 - .5)

    def test_gripper_is_deferred_until_motion_settles_and_timeout_cancels_it(self):
        robot = R5Adapter()
        robot.config = self.config
        robot.execution = ExecutionSettings.from_config(self.config)
        home = self.config.robot.mounts[0].home_positions_rad
        fingers = width_to_joints(self.config.robot, .041)
        robot.targets = {"left": home + fingers}
        def send(targets):
            robot.targets = targets
        action = Action((ArmCommand("left", "joint_position", home), ArmCommand("left", "gripper", (.06,))), self.config.control_dt_s)
        ready = Feedback("executing", metrics={"arms": {"left": {"motion_reached": False}}})
        with patch.object(robot, "_send_targets", side_effect=send), patch.object(robot, "_tracking_feedback", return_value=ready):
            robot.apply(action)
            self.assertEqual(robot.targets["left"][6:], fingers)
            robot.before_physics_step(1.)
            self.assertIn("left", robot.pending_grippers)
            ready.metrics["arms"]["left"]["motion_reached"] = True
            robot.before_physics_step(.1)
            self.assertIn("left", robot.pending_grippers)
            robot.before_physics_step(.2)
            self.assertFalse(robot.pending_grippers)
            self.assertEqual(robot.targets["left"][6:], width_to_joints(self.config.robot, .06))
            robot.apply(action)
            robot.on_action_timeout()
            self.assertFalse(robot.pending_grippers)


if __name__ == "__main__":
    unittest.main()
