"""Numerics, coordinate conventions, and fail-before-actuation contracts."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

from fixtures import robot_config
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.robots.audit import require_cartesian_sim_ready
from robot_tasks.robots.control import plan_targets, width_to_joints
from robot_tasks.robots.kinematics import IKError, R5Kinematics
from robot_tasks.robots.lula import R5LulaKinematics, TCP_FRAME, prepare_lula_assets
from robot_tasks.robots.transforms import (axis_rotation, origin_transform, pose_errors,
                                         pose_residual, pose_transform, transform_pose)
from robot_tasks.types import Action, ArmCommand, ArmState

ROOT = Path(__file__).resolve().parents[1]


def fake_lula(profile):
    """A preselected-output boundary double, not an implementation of IK."""
    reference = R5Kinematics(profile)
    solver = SimpleNamespace(base_frame_name=lambda: profile.base_link,
        frame_names=lambda: [TCP_FRAME],
        pose=lambda q, frame: SimpleNamespace(matrix=lambda: reference._forward_transform(q)))
    description = SimpleNamespace(kinematics=lambda: solver,
        num_c_space_coords=lambda: len(profile.joint_names), c_space_coord_name=lambda i: profile.joint_names[i])
    return SimpleNamespace(load_robot=Mock(return_value=description), Pose3=lambda matrix: matrix.copy(),
        CyclicCoordDescentIkConfig=lambda: SimpleNamespace(max_num_descents=100, ccd_max_iterations=10,
                                                          bfgs_max_iterations=60),
        compute_ik_ccd=Mock(return_value=SimpleNamespace(cspace_position=profile.mounts[0].home_positions_rad,
                                                        success=True, num_descents=1)))


class R5KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.config = robot_config()
        self.profile = self.config.robot
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.native = fake_lula(self.profile)
        self.kin = R5LulaKinematics(self.profile, self.directory, lula_module=self.native)
        self.home = self.profile.mounts[0].home_positions_rad
        self.targets = {m.name: m.home_positions_rad + width_to_joints(self.profile, .04) for m in self.profile.mounts}

    def test_zero_pose_matches_hand_summed_urdf_offsets(self):
        # At q=0, joint3's pi roll reverses the following y/z translations;
        # joint6 restores the tool orientation. This does not use the solver.
        expected = (.2402, .42, .2035, 1., 0., 0., 0.)
        np.testing.assert_allclose(self.kin.forward((0.,)*6, "left"), expected, atol=1e-6)

    def test_rpy_uses_fixed_axis_order(self):
        transform = origin_transform((1, 2, 3), (math.pi/2, 0, math.pi/2))
        np.testing.assert_allclose(transform @ [0, 1, 0, 1], [1, 2, 4, 1], atol=1e-12)

    def test_rotation_roundtrip_and_residual_at_pi(self):
        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 2, -3)):
            for angle in (0., 1e-9, .7, math.pi - 1e-8, math.pi):
                matrix = np.eye(4)
                matrix[:3, :3] = axis_rotation(axis, angle)
                pose = transform_pose(matrix)
                np.testing.assert_allclose(pose_transform(pose), matrix, atol=1e-8)
                self.assertAlmostEqual(np.linalg.norm(pose_residual(np.eye(4), matrix)[3:]), angle, places=7)
                inverse_sign = pose[:3] + tuple(-v for v in pose[3:])
                self.assertLess(pose_errors(pose, inverse_sign)[1], 1e-10)

    def test_forward_model_has_no_legacy_inverse_solver(self):
        self.assertFalse(hasattr(R5Kinematics(self.profile), "solve"))

    def test_ik_with_rotated_translated_mount_and_both_frames(self):
        base = (.4, -.3, .2, math.sqrt(.5), 0., 0., math.sqrt(.5))
        profile = replace(self.profile, mounts=(replace(self.profile.mounts[0], base_pose_wxyz=base), self.profile.mounts[1]))
        native = fake_lula(profile)
        kin = R5LulaKinematics(profile, self.directory, lula_module=native)
        q = (.15, 1.05, 1.3, .1, .15, -.1)
        native.compute_ik_ccd.return_value.cspace_position = q
        world = kin.forward(q, "left")
        local = kin.forward(q, "left", "robot_base")
        np.testing.assert_allclose(kin.world_pose(local, "left", "robot_base"), world, atol=1e-12)
        for frame, target in (("world", world), ("robot_base", local)):
            solution = kin.solve(target, "left", self.home, frame)
            self.assertTrue(solution.converged, solution)
            position, orientation = pose_errors(kin.forward(solution.positions_rad, "left"), world)
            self.assertLessEqual(position, kin.position_tolerance_m)
            self.assertLessEqual(orientation, kin.orientation_tolerance_rad)
            self.assertTrue(np.all(np.array(solution.positions_rad) >= kin.lower))
            self.assertTrue(np.all(np.array(solution.positions_rad) <= kin.upper))
            np.testing.assert_allclose(native.compute_ik_ccd.call_args.args[1], pose_transform(local), atol=1e-12)
            np.testing.assert_allclose(kin.ik_config.cspace_seeds[0], self.home)

    def test_outside_reach_and_local_nonconvergence_are_distinguished(self):
        far = self.kin.solve((3, 0, .5, 1, 0, 0, 0), "left", self.home)
        self.assertFalse(far.converged)
        self.assertEqual(far.reason, "outside_conservative_reach_bound")
        self.native.compute_ik_ccd.assert_not_called()
        goal = self.kin.forward((.4, .9, 1.5, .4, .4, .4), "left")
        self.native.compute_ik_ccd.return_value.success = False
        limited = self.kin.solve(goal, "left", self.home)
        self.assertFalse(limited.converged)
        self.assertEqual(limited.reason, "lula_not_converged")
        self.assertIsNone(limited.iterations)
        self.assertEqual(limited.descents, 1)

    def test_invalid_quaternion_frame_and_missing_tcp_are_rejected(self):
        for pose, frame in (((0, 0, 0, 0, 0, 0, 0), "world"),
                            ((float("nan"), 0, 0, 1, 0, 0, 0), "world"),
                            ((0, 0, 0, 1, 0, 0, 0), "camera")):
            with self.subTest(pose=pose, frame=frame), self.assertRaises(ValueError):
                self.kin.solve(pose, "left", self.home, frame)
        with self.assertRaisesRegex(ValueError, "TCP"):
            R5Kinematics(replace(self.profile, tcp_pose_wxyz=None))
        with self.assertRaisesRegex(ValueError, "TCP"):
            require_cartesian_sim_ready(replace(self.profile, tcp_source=""))

    def test_bimanual_cartesian_plan_preserves_gripper_and_input(self):
        q = (.15, 1.05, 1.3, .1, .15, -.1)
        self.native.compute_ik_ccd.return_value.cspace_position = q
        commands = tuple(ArmCommand(name, "cartesian_pose", self.kin.forward(q, name)) for name in self.config.arms)
        before = dict(self.targets)
        planned = plan_targets(self.profile, self.config.arms, Action(commands, .1), self.targets,
                               kinematics=self.kin, measured_positions={name: self.home for name in self.config.arms})
        self.assertEqual(self.targets, before)
        for name in self.config.arms:
            self.assertEqual(planned[name][6:], before[name][6:])
            self.assertLess(pose_errors(self.kin.forward(planned[name][:6], name), self.kin.forward(q, name))[0], 1e-4)

    def test_joint_and_cartesian_command_on_same_arm_conflict(self):
        commands = (ArmCommand("left", "joint_position", self.home),
                    ArmCommand("left", "cartesian_pose", self.kin.forward(self.home, "left")))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            plan_targets(self.profile, self.config.arms, Action(commands, .1), self.targets, kinematics=self.kin,
                         measured_positions={"left": self.home})

    def test_adapter_rejection_does_not_send_any_arm_to_physics(self):
        robot = R5Adapter()
        robot.config, robot.kinematics, robot.targets = self.config, self.kin, dict(self.targets)
        robot.instances = {name: Mock() for name in self.config.arms}
        states = {name: ArmState(joint_positions_rad=self.home) for name in self.config.arms}
        bad = Action((ArmCommand("left", "gripper", (.06,)),
                      ArmCommand("right", "cartesian_pose", (3, 0, .5, 1, 0, 0, 0))), .1)
        with patch.object(robot, "state", return_value=states), self.assertRaises(IKError):
            robot.apply(bad)
        for arm in robot.instances.values():
            arm.apply_action.assert_not_called()
        self.assertEqual(robot.targets, self.targets)
        self.assertEqual(robot.feedback().status, "rejected")

    def test_lula_nonconvergence_rejects_the_whole_bimanual_action(self):
        robot = R5Adapter()
        robot.config, robot.kinematics, robot.targets = self.config, self.kin, dict(self.targets)
        self.native.compute_ik_ccd.return_value.success = False
        states = {name: ArmState(joint_positions_rad=self.home) for name in self.config.arms}
        action = Action((ArmCommand("left", "gripper", (.06,)), ArmCommand(
            "right", "cartesian_pose", self.kin.forward(self.home, "right"))), .1)
        with patch.object(robot, "state", return_value=states), patch.object(robot, "_send_targets") as send:
            with self.assertRaises(IKError):
                robot.apply(action)
            send.assert_not_called()
        self.assertEqual(robot.targets, self.targets)
        self.assertEqual(robot.feedback().metrics["reason_code"], "lula_not_converged")

    def test_runtime_adapter_enables_lula_and_records_the_actual_solver(self):
        from robot_tasks.backends.isaacsim import IsaacSimBackend
        backend = IsaacSimBackend()
        backend.settings = SimpleNamespace(asset_directory=self.directory)
        extension = SimpleNamespace(enable_extension=Mock())
        robot = R5Adapter()
        with patch.dict(sys.modules, {"isaacsim.core.utils.extensions": extension}), \
                patch("robot_tasks.robots.lula.R5LulaKinematics", return_value=self.kin) as factory, \
                patch.object(backend, "add_r5", return_value={}):
            robot.initialize(backend, self.config)
        extension.enable_extension.assert_called_once_with("isaacsim.robot_motion.lula")
        factory.assert_called_once_with(self.profile, self.directory, sampling_seed=self.config.seed)
        self.assertIs(robot.kinematics, self.kin)
        self.assertEqual(backend.scene_report["ik_solver"], "lula")
        self.assertEqual(backend.scene_report["ik_configuration"]["tcp_frame"], TCP_FRAME)

    def test_generated_assets_follow_limits_and_rotated_tcp_without_touching_vendor(self):
        original = self.profile.asset_path.read_bytes()
        for angles in ((.2, -.3, .4), (.2, math.pi / 2, .4), (.2, -math.pi / 2, .4)):
            profile = replace(self.profile, tcp_pose_wxyz=transform_pose(origin_transform((.14, .01, .02), angles)))
            geometry = R5Kinematics(profile)
            urdf, yaml = prepare_lula_assets(geometry, self.directory)
            root = ET.parse(urdf).getroot()
            self.assertFalse(root.findall(".//mesh"))
            fixed = root.find("joint[@name='robot_tasks_tcp_fixed']")
            origin = fixed.find("origin")
            actual = origin_transform(tuple(map(float, origin.get("xyz").split())),
                                      tuple(map(float, origin.get("rpy").split())))
            np.testing.assert_allclose(actual, geometry.tcp, atol=1e-9)
            for limit in profile.operating_limits[:6]:
                joint = root.find(f"joint[@name='{limit.joint}']/limit")
                self.assertEqual(float(joint.get("lower")), limit.lower)
                self.assertEqual(float(joint.get("upper")), limit.upper)
            self.assertIn('"joint1"', yaml.read_text())
            self.assertEqual(prepare_lula_assets(geometry, self.directory), (urdf, yaml))
        self.assertEqual(self.profile.asset_path.read_bytes(), original)
        self.kin.description_path.write_text("changed", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "modified"):
            prepare_lula_assets(self.kin, self.directory)

    def test_sdk_success_does_not_bypass_limits_or_independent_tcp_error(self):
        target = self.kin.forward(self.home, "left")
        self.native.compute_ik_ccd.return_value.cspace_position = (3.,) + self.home[1:]
        result = self.kin.solve(target, "left", self.home)
        self.assertFalse(result.converged)
        self.assertEqual(result.reason, "lula_joint_limit_violation")
        self.native.compute_ik_ccd.return_value.cspace_position = (.1,) + self.home[1:]
        result = self.kin.solve(target, "left", self.home)
        self.assertFalse(result.converged)
        self.assertEqual(result.reason, "lula_residual_tolerance_exceeded")
        for invalid in ((float("nan"),) * 6, (0.,) * 5):
            self.native.compute_ik_ccd.return_value.cspace_position = invalid
            with self.assertRaisesRegex(RuntimeError, "invalid joint"):
                self.kin.solve(target, "left", self.home)

    def test_model_mismatch_and_missing_lula_fail_without_fallback(self):
        native = fake_lula(self.profile)
        native.load_robot.return_value.c_space_coord_name = lambda i: "wrong_joint"
        with self.assertRaisesRegex(RuntimeError, "joint order"):
            R5LulaKinematics(self.profile, self.directory, lula_module=native)
        native = fake_lula(self.profile)
        native.load_robot.return_value.kinematics().pose = lambda q, frame: SimpleNamespace(matrix=lambda: np.eye(4))
        with self.assertRaisesRegex(RuntimeError, "FK differs"):
            R5LulaKinematics(self.profile, self.directory, lula_module=native)
        with patch("robot_tasks.robots.lula.importlib.import_module", side_effect=ModuleNotFoundError("lula")):
            with self.assertRaisesRegex(RuntimeError, "Lula is required"):
                R5LulaKinematics(self.profile, self.directory)

    def test_near_limit_measured_seed_is_bounded_but_solution_is_not_clipped(self):
        self.kin.solve(self.kin.forward(self.home, "left"), "left", (2.51,) + self.home[1:])
        self.assertEqual(self.kin.ik_config.cspace_seeds[0][0], 2.5)

    def test_cartesian_completion_requires_measured_tcp_accuracy(self):
        robot = R5Adapter()
        robot.config = self.config
        robot.targets = self.targets
        target = self.kin.forward(self.home, "left")
        robot.cartesian_targets = {"left": target}
        states = {name: ArmState(joint_positions_rad=self.home, finger_positions_m=q[6:],
                                 joint_velocities_rad_s=(0.,)*6, finger_velocities_m_s=(0.,)*2,
                                 tcp_pose_world_wxyz=self.kin.forward(self.home, name))
                  for name, q in self.targets.items()}
        with patch.object(robot, "state", return_value=states):
            self.assertEqual(robot.feedback().status, "completed")
            states["left"] = replace(states["left"], tcp_pose_world_wxyz=(target[0]+.01,) + target[1:])
            self.assertEqual(robot.feedback().status, "executing")

    def test_holding_existing_targets_does_not_resend_or_wake_either_arm(self):
        robot = R5Adapter()
        robot.config, robot.targets = self.config, dict(self.targets)
        robot.instances = {name: Mock() for name in self.config.arms}
        robot.apply(Action((), .1))
        robot.apply(Action((ArmCommand("left", "joint_position", self.home),), .1))
        for arm in robot.instances.values():
            arm.apply_action.assert_not_called()
        self.assertEqual(robot.targets, self.targets)


if __name__ == "__main__":
    unittest.main()
