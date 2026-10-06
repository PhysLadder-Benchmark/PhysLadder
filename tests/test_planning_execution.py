"""Offline checks of trajectory bounds, atomic rejection and gripper semantics."""

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from fixtures import robot_config
from robot_tasks.config import ExecutionSettings
from robot_tasks.planning.trajectory import JointPath
from robot_tasks.robots.control import (
    bounded_measured_joints,
    normalized_to_width,
    width_to_normalized,
    width_to_joints,
)
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.types import ArmState, Action, ArmCommand, Feedback


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

    def test_checked_timing_survives_controller_validation(self):
        start = (0.,) * 6
        path = JointPath.from_waypoints([start, (.1, 0, 0, 0, 0, 0)], start, self.bounds)
        slow = path.stretched(2.)
        loaded = JointPath.from_timed(slow.positions, slow.times_s, start, self.bounds)
        self.assertEqual(loaded, slow)
        self.assertAlmostEqual(loaded.sample(1.)[0], .05)
        with self.assertRaises(ValueError):
            JointPath.from_timed(path.positions, [t / 10 for t in path.times_s], start, self.bounds)
        with self.assertRaises(ValueError):
            JointPath.from_timed(path.positions, list(reversed(path.times_s)), start, self.bounds)
        with self.assertRaises(ValueError):
            path.stretched(.001)

    def test_stationary_hand_can_share_the_other_hand_duration(self):
        q = (0.,) * 6
        hold = JointPath((q,), (0.,)).stretched(2.)
        self.assertEqual(JointPath.from_timed(hold.positions, hold.times_s, q, self.bounds), hold)
        self.assertEqual(hold.sample(1.5), q)

    def test_adapter_keeps_worker_timing_instead_of_regenerating_it(self):
        robot, states = self.robot()
        robot.execution = replace(ExecutionSettings.from_config(self.config), max_action_steps=120)
        start = states['left'].joint_positions_rad
        end = (start[0] + .03, *start[1:])
        path = JointPath.from_waypoints([start, end], start, self.bounds).stretched(2.)
        robot.backend, robot.planner = Mock(), Mock()
        robot.planner.plan.return_value = {'paths': {'left': {
            'positions': path.positions, 'times_s': path.times_s}}, 'diagnostics': {}}
        with patch('robot_tasks.planning.scene.snapshot', return_value={}), patch.object(robot, '_send_targets'):
            robot.apply(Action((ArmCommand('left', 'joint_position', end),), self.config.control_dt_s))
        self.assertEqual(robot.paths['left'], path)

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

    def test_timeout_float32_limit_noise_does_not_block_gripper_recovery(self):
        robot, states = self.robot()
        q = list(states["right"].joint_positions_rad)
        q[1] = 2.5000014305114746  # Recorded L2-04 PhysX measurement under load.
        states["right"] = replace(states["right"], joint_positions_rad=tuple(q))
        robot.motion_arms = {"right"}
        def send(targets):
            robot.targets = dict(targets)
        with patch.object(robot, "_send_targets", side_effect=send):
            robot.on_action_timeout()
            self.assertEqual(robot.targets["right"][1], 2.5)
            robot.apply(Action((ArmCommand("right", "gripper", (.07,)),),
                               self.config.control_dt_s))
        self.assertEqual(states["right"].joint_positions_rad[1], q[1])
        self.assertEqual(robot.targets["right"][6:], width_to_joints(self.config.robot, .07))

    def test_trajectory_does_not_reintroduce_measured_limit_noise(self):
        actual = (0., 2.5000014305114746, 1.2, 0., 0., 0.)
        bounded = (0., 2.5, 1.2, 0., 0., 0.)
        end = (0., 2.49, 1.2, 0., 0., 0.)
        path = JointPath.from_waypoints([bounded, end], actual, self.bounds)
        self.assertEqual(path.positions[0], bounded)
        self.assertTrue(all(row[1] <= 2.5 for row in path.positions))
        # A genuinely out-of-range state or requested path must still fail.
        with self.assertRaises(ValueError):
            JointPath.from_waypoints([bounded, end], (0., 2.5001, 1.2, 0., 0., 0.), self.bounds)
        with self.assertRaises(ValueError):
            JointPath.from_waypoints([actual, end], actual, self.bounds)

    def test_accepted_wait_clears_old_rejection_without_changing_drives(self):
        robot, _ = self.robot()
        robot.rejection = Feedback("rejected", "previous invalid target")
        robot.last_plan = {"paths": {"right": {"positions": [[0.] * 6]}}}
        before = dict(robot.targets)
        with patch.object(robot, "_send_targets") as send:
            robot.apply(Action((), self.config.control_dt_s))
        send.assert_not_called()
        self.assertEqual(robot.targets, before)
        self.assertNotEqual(robot.feedback().status, "rejected")
        self.assertIsNone(robot.last_plan)

    def test_only_measured_roundoff_is_bounded_not_real_limit_violations(self):
        q = (-2.5000016689300537, 2.5000014305114746, 0., 0., 0., 0.)
        self.assertEqual(bounded_measured_joints(q, self.bounds), (-2.5, 2.5, 0., 0., 0., 0.))
        for bad in ((-2.5001,) + q[1:], q[:1], (float("nan"),) + q[1:]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                bounded_measured_joints(bad, self.bounds)

    def test_planner_uses_bounded_start_and_preserves_raw_observation(self):
        from robot_tasks.planning.worker import CuroboWorker

        worker = CuroboWorker.__new__(CuroboWorker)
        worker.profile = self.config.robot
        worker.torch = SimpleNamespace(
            tensor=lambda values, **kwargs: np.asarray(values), float32=np.float32,
        )
        worker.spheres = Mock(return_value={self.config.robot.ee_link: []})
        worker.world = Mock(return_value={})
        worker.check_payload = Mock()
        worker.planner = Mock()
        raw = (0., 2.5000016689300537, 1.2, 0., 0., 0.)
        start = (0., 2.5, 1.2, 0., 0., 0.)
        end = (0., 2.49, 1.2, 0., 0., 0.)
        trajectory = Mock(joint_names=list(self.config.robot.joint_names))
        trajectory.position.detach().cpu().numpy.return_value = np.array([start, end])
        result = Mock(success=np.array([True]))
        result.get_interpolated_plan.return_value = trajectory
        worker.planner.plan_cspace.return_value = result
        types = SimpleNamespace(
            JointState=SimpleNamespace(from_position=lambda q, **kwargs: q),
            GoalToolPose=Mock(),
        )
        request = {"id": 1, "commands": [{"arm": "right", "kind": "joint_position", "values": end}],
                   "states": {"right": {"joint_positions_rad": raw}}}
        with patch.dict("sys.modules", {"curobo.types": types}):
            reply = worker.plan(request)
        np.testing.assert_array_equal(worker.planner.plan_cspace.call_args.args[1], [start])
        self.assertEqual(request["states"]["right"]["joint_positions_rad"], raw)
        self.assertGreater(reply["diagnostics"]["right"]["start_joint_limit_correction_max_rad"], 0.)
        self.assertEqual(reply["paths"]["right"]["positions"][0], start)
        # The same production worker must align arrival times before checking
        # inter-arm collisions, including different travel distances.
        long_trajectory = Mock(joint_names=list(self.config.robot.joint_names))
        long_end = (0., 2.3, 1.2, 0., 0., 0.)
        long_trajectory.position.detach().cpu().numpy.return_value = np.array([start, long_end])
        long_result = Mock(success=np.array([True]))
        long_result.get_interpolated_plan.return_value = long_trajectory
        worker.planner.plan_cspace.side_effect = [long_result, result]
        worker.check_bimanual = Mock()
        request['states']['left'] = {'joint_positions_rad': raw}
        request['commands'].insert(0, {'arm': 'left', 'kind': 'joint_position', 'values': long_end})
        with patch.dict('sys.modules', {'curobo.types': types}):
            paired = worker.plan(request)
        self.assertAlmostEqual(paired['paths']['left']['times_s'][-1], paired['paths']['right']['times_s'][-1])
        self.assertGreater(paired['paths']['right']['times_s'][-1], reply['paths']['right']['times_s'][-1])
        worker.check_bimanual.assert_called_once_with(paired['paths'], request['states'])


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
