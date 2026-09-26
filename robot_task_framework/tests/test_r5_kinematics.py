"""Numerics, coordinate conventions, and fail-before-actuation contracts."""

from dataclasses import replace
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np

from robot_tasks.config import load_config
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.robots.audit import require_cartesian_sim_ready
from robot_tasks.robots.control import plan_targets, width_to_joints
from robot_tasks.robots.kinematics import IKError, R5Kinematics
from robot_tasks.robots.transforms import (axis_rotation, origin_transform, pose_errors,
                                         pose_residual, pose_transform, transform_pose)
from robot_tasks.types import Action, ArmCommand, ArmState

ROOT = Path(__file__).resolve().parents[1]


class R5KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "configs/checks/r5_cartesian.toml")
        self.profile = self.config.robot
        self.kin = R5Kinematics(self.profile)
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

    def test_geometric_jacobian_matches_finite_differences(self):
        q = np.array([.2, 1.1, 1.3, -.1, .2, .3])
        current, jacobian = self.kin._forward_jacobian(q)
        for i in range(6):
            step = q.copy()
            step[i] += 1e-7
            moved, _ = self.kin._forward_jacobian(step)
            np.testing.assert_allclose(jacobian[:, i], pose_residual(current, moved) / 1e-7, atol=1e-7)

    def test_ik_with_rotated_translated_mount_and_both_frames(self):
        base = (.4, -.3, .2, math.sqrt(.5), 0., 0., math.sqrt(.5))
        profile = replace(self.profile, mounts=(replace(self.profile.mounts[0], base_pose_wxyz=base), self.profile.mounts[1]))
        kin = R5Kinematics(profile)
        q = (.15, 1.05, 1.3, .1, .15, -.1)
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

    def test_outside_reach_and_local_nonconvergence_are_distinguished(self):
        far = self.kin.solve((3, 0, .5, 1, 0, 0, 0), "left", self.home)
        self.assertFalse(far.converged)
        self.assertEqual(far.reason, "outside_conservative_reach_bound")
        goal = self.kin.forward((.4, .9, 1.5, .4, .4, .4), "left")
        limited = self.kin.solve(goal, "left", self.home, max_iterations=1)
        self.assertFalse(limited.converged)
        self.assertEqual(limited.reason, "local_solver_not_converged")

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

    def test_cartesian_completion_requires_measured_tcp_accuracy(self):
        robot = R5Adapter()
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
