"""Offline checks of trajectory bounds, atomic rejection and gripper semantics."""

from dataclasses import replace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from fixtures import robot_config
from robot_tasks.planning.trajectory import JointPath
from robot_tasks.robots.control import (
    normalized_to_width,
    width_to_normalized,
    width_to_joints,
)
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.types import ArmState, Action, ArmCommand


class PlanningExecutionTests(unittest.TestCase):
    def setUp(self):
        self.config = robot_config()
        self.bounds = [
            x
            for x in self.config.robot.operating_limits
            if x.joint in self.config.robot.joint_names
        ]

    def test_normalized_gripper_endpoints_and_invalid_values(self):
        for x, w in ((0, 0.012), (0.5, 0.041), (1, 0.07)):
            self.assertAlmostEqual(normalized_to_width(self.config.robot, x), w)
            self.assertAlmostEqual(width_to_normalized(self.config.robot, w), x)
        for x in (-0.1, 1.1, True, float("nan")):
            with self.assertRaises(ValueError):
                normalized_to_width(self.config.robot, x)

    def test_path_preserves_detour_and_caps_each_joint_increment_and_speed(self):
        a = (0.0,) * 6
        b = (0.2, 0.1, 0, 0, 0, 0)
        c = (0.1, 0, 0, 0, 0, 0)
        path = JointPath.from_waypoints([a, b, c], a, self.bounds)
        q = np.array(path.positions)
        dt = np.diff(path.times_s)
        self.assertTrue(any(np.allclose(row, b) for row in q))
        self.assertLessEqual(np.max(abs(np.diff(q, axis=0))), 0.01000001)
        self.assertTrue(np.all(abs(np.diff(q, axis=0)) / dt[:, None] <= 0.50000001))
        for invalid in ([[3] * 6], [[float("nan")] * 6], [[1] * 5], [[0.1] * 6]):
            with self.assertRaises(ValueError):
                JointPath.from_waypoints(invalid, a, self.bounds)

    def robot(self):
        robot = R5Adapter()
        robot.config = self.config
        home = self.config.robot.mounts[0].home_positions_rad
        robot.targets = {
            a: home + width_to_joints(self.config.robot, 0.012)
            for a in self.config.arms
        }
        states = {
            a: ArmState(
                joint_positions_rad=home,
                joint_velocities_rad_s=(0.0,) * 6,
                gripper_width_m=0.035,
                finger_positions_m=(0.0175, 0.0175),
                finger_velocities_m_s=(0.0, 0.0),
            )
            for a in self.config.arms
        }
        robot.state = Mock(return_value=states)
        return robot, states

    def test_stationary_blocked_close_completes_only_after_dwell_without_grasp_claim(
        self,
    ):
        robot, _ = self.robot()
        for a in self.config.arms:
            robot.grip_closing[a] = True
            robot.grip_elapsed[a] = 0.0
            robot.grip_durations[a] = 1.0
        self.assertEqual(robot.feedback().status, "executing")
        for a in self.config.arms:
            robot.grip_elapsed[a] = 1.1
        feedback = robot.feedback()
        self.assertEqual(feedback.status, "completed")
        detail = feedback.metrics["arms"]["left"]
        self.assertFalse(detail["gripper_reached"])
        self.assertIsNone(detail["grasp_confirmed"])
        self.assertEqual(detail["gripper_state"], "closing_obstructed_or_stalled")
        self.assertAlmostEqual(detail["measured_gripper_width_m"], 0.035)

    def test_opening_stalled_does_not_use_close_obstruction_exception(self):
        robot, _ = self.robot()
        robot.targets = {
            a: q[:6] + width_to_joints(self.config.robot, 0.07)
            for a, q in robot.targets.items()
        }
        robot.grip_elapsed = {a: 100.0 for a in self.config.arms}
        robot.grip_durations = {a: 1.0 for a in self.config.arms}
        self.assertEqual(robot.feedback().status, "executing")

    def test_planning_failure_does_not_apply_even_other_arm_gripper(self):
        robot, states = self.robot()
        before = dict(robot.targets)
        robot.backend = Mock()
        robot.planner = Mock()
        robot.planner.plan.side_effect = ValueError("no path")
        action = Action(
            (
                ArmCommand("left", "gripper", (0.07,)),
                ArmCommand(
                    "right", "joint_position", states["right"].joint_positions_rad
                ),
            ),
            self.config.control_dt_s,
        )
        with (
            patch("robot_tasks.planning.scene.snapshot", return_value={}),
            patch.object(robot, "_send_targets") as send,
        ):
            with self.assertRaises(ValueError):
                robot.apply(action)
            send.assert_not_called()
        self.assertEqual(robot.targets, before)
        self.assertEqual(robot.feedback().status, "rejected")

    def test_timeout_cancels_cartesian_goal_and_holds_measured_pose(self):
        robot, states = self.robot()
        robot.motion_arms = {"left"}
        robot.cartesian_targets = {"left": (0.3, 0.4, 0.2, 1.0, 0.0, 0.0, 0.0)}
        robot.pending_grippers = {"left": (0.035, 0.035)}
        with patch.object(robot, "_send_targets") as send:
            robot.on_action_timeout()
        self.assertFalse(robot.cartesian_targets)
        self.assertFalse(robot.pending_grippers)
        target = send.call_args.args[0]["left"]
        self.assertEqual(target[:6], states["left"].joint_positions_rad)
        self.assertEqual(target[6:], robot.targets["left"][6:])


class PayloadClearanceTests(unittest.TestCase):
    def test_midpath_fixture_collision_is_rejected_but_lifted_path_is_allowed(self):
        try:
            import trimesh
            import rtree
        except ImportError:
            self.skipTest(
                "Mesh proximity dependencies are supplied by the planner environment"
            )
        from robot_tasks.planning.worker import CuroboWorker
        from types import SimpleNamespace
        from robot_tasks.planning.scene import box_surface

        worker = CuroboWorker.__new__(CuroboWorker)
        worker.kin = SimpleNamespace(
            forward=lambda q, arm: (q[0], 0.0, q[1], 1.0, 0.0, 0.0, 0.0)
        )
        mesh = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
        points = box_surface([0.04] * 3) + [0.0, 0.0, 0.1]
        request = {
            "states": {"left": {"joint_positions_rad": [0.0, 0.1]}},
            "scene": {
                "payloads": {"left": [{"surface_points_world": points.tolist()}]},
                "obstacles": {
                    "mesh": {
                        "fixture": {
                            "pose": [0.5, 0, 0.1, 1, 0, 0, 0],
                            "vertices": mesh.vertices.tolist(),
                            "faces": mesh.faces.tolist(),
                        }
                    }
                },
            },
        }
        blocked = JointPath(((0.0, 0.1), (1.0, 0.1)), (0.0, 1.0))
        with self.assertRaisesRegex(ValueError, "penetrates fixture"):
            worker.check_payload("left", blocked, request, {})
        clear = JointPath(
            ((0.0, 0.1), (0.0, 0.3), (1.0, 0.3), (1.0, 0.1)), (0.0, 0.2, 1.2, 1.4)
        )
        diagnostic = {}
        worker.check_payload("left", clear, request, diagnostic)
        self.assertIn("held surface samples", diagnostic["payload_guard"])
