"""Regression for the recorded L1 first-step failure; no model or simulator."""

from dataclasses import replace
import math
import tempfile
from pathlib import Path
from importlib.metadata import PackageNotFoundError
import unittest
from unittest.mock import patch

import numpy as np

from robot_tasks.config import ExecutionSettings
from robot_tasks.registry import resolve_task
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.robots.control import width_to_joints
from robot_tasks.robots.gripper import GripperProgress
from robot_tasks.robots.lula import R5LulaKinematics
from robot_tasks.integrationtest.lula_check import load_native_lula
from robot_tasks.robots.motion import BenchGuard, interpolated_poses, plan_cartesian, JointPath
from robot_tasks.robots.transforms import pose_errors
from robot_tasks.types import Action, ArmCommand, ArmState, Feedback


class MotionPathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.config = resolve_task("L1-01")
        try:
            native = load_native_lula()
        except (PackageNotFoundError, ModuleNotFoundError) as exc:
            raise unittest.SkipTest("Native Lula path checks require Isaac Python") from exc
        directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(directory.cleanup)
        cls.kin = R5LulaKinematics(cls.config.robot, Path(directory.name), lula_module=native)
        cls.guard = BenchGuard(cls.config.robot)
        cls.home = cls.config.robot.mounts[0].home_positions_rad

    def test_recorded_first_target_has_a_continuous_bench_clear_path(self):
        goal = (0.4, 0.06, 0.16, 1, 0, 0, 0)
        path = plan_cartesian(self.kin, self.guard, "left", self.home, goal, (0.035, 0.035), budget_s=9)
        self.assertGreater(len(path.points), 20)
        self.assertLess(path.times[-1], 9)
        velocity = abs(np.diff(path.points, axis=0)) / np.diff(path.times)[:, None]
        self.assertLessEqual(float(velocity.max()), 0.500001)
        start = np.array(self.kin.forward(self.home, "left")[:3])
        goal_xyz = np.array(goal[:3])
        for i, q in enumerate(path.points):
            actual = self.kin.forward(q, "left")
            expected_xyz = start + i / (len(path.points) - 1) * (goal_xyz - start)
            np.testing.assert_allclose(actual[:3], expected_xyz, atol=1e-4)
            self.assertLess(pose_errors(actual, (*actual[:3], *goal[3:]))[1], 1e-3)

    def test_downward_pose_through_table_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "table_collision"):
            plan_cartesian(
                self.kin,
                self.guard,
                "left",
                self.home,
                (0.4, 0.42, 0.0, math.sqrt(0.5), 0, math.sqrt(0.5), 0),
                (0.035, 0.035),
                budget_s=9,
            )

    def test_path_longer_than_budget_is_rejected_not_sped_up(self):
        with self.assertRaisesRegex(ValueError, "motion_budget"):
            plan_cartesian(
                self.kin,
                self.guard,
                "left",
                self.home,
                (0.4, 0.06, 0.16, 1, 0, 0, 0),
                (0.035, 0.035),
                budget_s=0.1,
            )


class PathPrimitiveTests(unittest.TestCase):
    def test_quaternion_interpolation_uses_short_arc(self):
        poses, distance, angle = interpolated_poses((0, 0, 0, 1, 0, 0, 0), (0, 0, 0.1, -1, 0, 0, 0))
        self.assertAlmostEqual(angle, 0)
        for p in poses:
            self.assertAlmostEqual(np.linalg.norm(p[3:]), 1)

    def test_reference_stops_advancing_when_physics_cannot_follow(self):
        path = JointPath(np.array([[0.0] * 6, [0.5] * 6]), np.array([0.0, 1.0]))
        for _ in range(100):
            q = path.advance(0.01, np.zeros(6))
        self.assertLessEqual(max(q), 0.086)
        self.assertFalse(path.finished)


class GripperSequenceTests(unittest.TestCase):
    def setUp(self):
        _, self.config = resolve_task("L1-01")
        self.robot = R5Adapter()
        self.robot.config = self.config
        self.robot.execution = replace(ExecutionSettings.from_config(self.config), motion_mode="direct_ik")
        self.home = self.config.robot.mounts[0].home_positions_rad
        self.fingers = width_to_joints(self.config.robot, 0.041)
        self.robot.targets = {"left": self.home + self.fingers}
        self.states = {"left": ArmState(self.home, 0.041, (0.0,) * 6, self.fingers, (0.0,) * 2)}
        self.p1 = patch.object(self.robot, "state", side_effect=lambda: self.states)
        self.p2 = patch.object(self.robot, "_send_targets", side_effect=self.send)
        self.p1.start()
        self.p2.start()
        self.addCleanup(self.p1.stop)
        self.addCleanup(self.p2.stop)

    def send(self, targets):
        self.robot.targets = targets

    def combined(self, width):
        q = (self.home[0] + 0.1,) + self.home[1:]
        return Action(
            (ArmCommand("left", "joint_position", q), ArmCommand("left", "gripper", (width,))),
            self.config.control_dt_s,
        )

    def test_open_first_holds_arm_until_fingers_arrive(self):
        self.robot.apply(self.combined(0.07))
        self.assertEqual(self.robot.targets["left"][:6], self.home)
        self.assertEqual(self.robot.targets["left"][6:], (0.035, 0.035))
        self.robot.before_physics_step(0.1)
        self.assertTrue(self.robot.opening_arm_targets)
        self.assertEqual(self.robot.targets["left"][:6], self.home)
        self.states["left"] = replace(
            self.states["left"], gripper_width_m=0.07, finger_positions_m=(0.035, 0.035)
        )
        self.robot.before_physics_step(0.1)
        self.assertFalse(self.robot.opening_arm_targets)
        self.assertAlmostEqual(self.robot.targets["left"][0], self.home[0] + 0.1)

    def test_closing_still_waits_for_motion_to_settle(self):
        self.robot.apply(self.combined(0.02))
        self.assertEqual(self.robot.targets["left"][6:], self.fingers)
        feedback = Feedback("executing", metrics={"arms": {"left": {"motion_reached": False}}})
        with patch.object(self.robot, "_tracking_feedback", return_value=feedback):
            self.robot.before_physics_step(1.0)
            self.assertTrue(self.robot.pending_grippers)
            feedback.metrics["arms"]["left"]["motion_reached"] = True
            self.robot.before_physics_step(0.3)
        self.assertEqual(self.robot.targets["left"][6:], (0.01, 0.01))

    def test_timeout_cancels_opening_motion_and_later_wait_cannot_resume_it(self):
        self.robot.apply(self.combined(0.07))
        self.robot.on_action_timeout()
        self.assertFalse(self.robot.opening_arm_targets)
        self.robot.apply(Action((), self.config.control_dt_s))
        self.robot.before_physics_step(1.0)
        self.assertEqual(self.robot.targets["left"][:6], self.home)

    def test_timeout_cancels_waypoints_and_holds_measured_joints(self):
        self.robot.execution = replace(self.robot.execution, motion_mode="cartesian_linear_v1")
        self.robot.motion_arms = {"left"}
        self.robot.motion_paths = {
            "left": JointPath(np.array([self.home, np.array(self.home) + 0.1]), np.array([0.0, 1.0]))
        }
        self.robot.cartesian_targets = {"left": (0.4, 0.1, 0.2, 1, 0, 0, 0)}
        cancellation = self.robot.on_action_timeout()
        self.assertTrue(cancellation["pending_motion_cancelled"])
        self.assertFalse(self.robot.motion_paths)
        self.assertFalse(self.robot.cartesian_targets)
        self.robot.before_physics_step(1.0)
        self.assertEqual(self.robot.targets["left"][:6], self.home)

    def test_rejected_motion_cannot_open_gripper_or_replace_existing_queue(self):
        self.robot.apply(self.combined(0.07))
        before = dict(self.robot.targets)
        queue = dict(self.robot.opening_arm_targets)
        bad = Action(
            (ArmCommand("left", "gripper", (0.012,)), ArmCommand("left", "joint_position", (100.0,) * 6)), 0.1
        )
        with self.assertRaises(ValueError):
            self.robot.apply(bad)
        self.assertEqual(self.robot.targets, before)
        self.assertEqual(self.robot.opening_arm_targets, queue)

    def test_repeat_width_after_disturbance_does_not_block_retreat_on_reopening(self):
        self.robot.gripper_intents["left"] = .03
        fingers = width_to_joints(self.config.robot, .026)
        self.states["left"] = replace(self.states["left"], gripper_width_m=.026, finger_positions_m=fingers)
        self.robot.apply(self.combined(.03))
        self.assertFalse(self.robot.opening_arm_targets)
        self.assertAlmostEqual(self.robot.targets["left"][0], self.home[0] + .1)

    def test_stalled_closing_completes_without_claiming_grasp_and_retains_drive(self):
        self.robot.apply(Action((ArmCommand("left", "gripper", (.02,)),), self.config.control_dt_s))
        for _ in range(8):
            self.robot.before_physics_step(.1)
        feedback = self.robot.feedback()
        self.assertEqual(feedback.status, "completed")
        arm = feedback.metrics["arms"]["left"]
        self.assertEqual(arm["gripper_state"], "closing_stalled")
        self.assertFalse(arm["gripper_reached"])
        self.assertIsNone(arm["grasp_confirmed"])
        self.assertEqual(self.robot.targets["left"][6:], (.01, .01))

    def test_legacy_position_mode_does_not_complete_blocked_closing(self):
        self.robot.execution = replace(self.robot.execution, gripper_completion="position")
        self.robot.apply(Action((ArmCommand("left", "gripper", (.02,)),), .1))
        for _ in range(10):
            self.robot.before_physics_step(.1)
        self.assertEqual(self.robot.feedback().status, "executing")
        self.assertFalse(self.robot.feedback().metrics["arms"]["left"]["gripper_reached"])

    def test_same_width_during_carry_does_not_queue_another_close(self):
        self.robot.gripper_intents["left"] = .02
        self.robot.targets["left"] = self.home + (.01,.01)
        self.robot.apply(self.combined(.02))
        self.assertFalse(self.robot.pending_grippers)
        self.assertFalse(self.robot.opening_arm_targets)
        self.assertEqual(self.robot.targets["left"][6:], (.01,.01))

    def test_stalled_opening_cancels_queue_and_allows_explicit_arm_only_retreat(self):
        self.robot.apply(self.combined(.07))
        for _ in range(8):
            self.robot.before_physics_step(.1)
        self.assertEqual(self.robot.feedback().status, "blocked")
        self.assertFalse(self.robot.opening_arm_targets)
        self.assertFalse(self.robot.motion_paths)
        self.assertEqual(self.robot.targets["left"][:6], self.home)
        self.robot.apply(Action((), self.config.control_dt_s))
        self.robot.before_physics_step(.2)
        self.assertEqual(self.robot.targets["left"][:6], self.home)
        retreat = self.combined(.07).commands[0]
        self.robot.apply(Action((retreat,), self.config.control_dt_s))
        self.assertIsNone(self.robot.blocked_feedback)
        self.assertFalse(self.robot.opening_arm_targets)
        self.assertEqual(self.robot.targets["left"][:6], retreat.values)


class GripperProgressTests(unittest.TestCase):
    def test_contact_velocity_chatter_with_fixed_positions_is_stalled(self):
        monitor = GripperProgress()
        for i in range(10):
            monitor.update((.02,.02), ((-1)**i * .03, .03), (.01,.01), .1)
        self.assertEqual(monitor.status, "closing_stalled")
    def test_slow_progress_is_not_a_stall(self):
        monitor = GripperProgress()
        for i in range(50):
            q = .03 - i * .0001
            monitor.update((q,q), (-.001,-.001), (.01,.01), .1)
            self.assertEqual(monitor.status, "moving")

    def test_change_of_target_resets_stall_window(self):
        monitor = GripperProgress()
        for _ in range(8):
            monitor.update((.02,.02), (0.,0.), (.01,.01), .1)
        self.assertEqual(monitor.status, "closing_stalled")
        monitor.update((.02,.02), (0.,0.), (.035,.035), .1)
        self.assertEqual(monitor.status, "moving")

    def test_exact_position_with_velocity_is_not_settled(self):
        monitor = GripperProgress()
        monitor.update((.02,.02), (.01,.01), (.02,.02), .1)
        self.assertEqual(monitor.status, "moving")


if __name__ == "__main__":
    unittest.main()
