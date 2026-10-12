import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from robot_tasks.robots.collision_feedback import CollisionMonitor, unexpected_pair
from robot_tasks.robots.adapters import R5Adapter


class CollisionFeedbackTests(unittest.TestCase):
    mounts = {"left": "/World/Robots/R5_Left", "right": "/World/Robots/R5_Right"}

    def test_fingers_can_grasp_but_housing_and_mount_cannot_push_props(self):
        root = self.mounts["right"]
        self.assertIsNone(unexpected_pair(root + "/link7/collisions/finger", "/World/Box", self.mounts))
        for part in ("Sensor/Housing", "Mount/Geometry"):
            row = unexpected_pair("/World/Box", root + "/link6/WristSensor/" + part, self.mounts)
            self.assertEqual(row["component"], "wrist_camera_or_mount")
            self.assertEqual(row["arm"], "right")

    def test_other_arm_collision_includes_fingers(self):
        left, right = (self.mounts[k] for k in ("left", "right"))
        self.assertIsNotNone(unexpected_pair(left + "/link7/c", right + "/link8/c", self.mounts))
        self.assertIsNone(unexpected_pair(left + "/link6/c", left + "/link7/c", self.mounts))
        self.assertIsNone(unexpected_pair("/World/Brush/c", "/World/Box/c", self.mounts))

    def monitor(self):
        monitor = CollisionMonitor.__new__(CollisionMonitor)
        monitor.reset()
        return monitor

    def test_impulse_uses_physics_dt_and_latches_until_next_action(self):
        m = self.monitor()
        m.pending = [{"impulse_ns": .0002, "separation_m": -.0001}]
        m.after_step(.001)
        self.assertAlmostEqual(m.blocked["force_n"], .2)
        m.after_step(.001)
        self.assertAlmostEqual(m.blocked["force_n"], .2)
        m.begin_action()
        self.assertIsNone(m.blocked)
        m.pending = [{"impulse_ns": .000001, "separation_m": .0005}]
        m.after_step(.001)
        self.assertIsNone(m.blocked)

    def test_nonfinite_contact_is_invalid_not_a_model_failure(self):
        from robot_tasks.types import InvalidSimulationState
        m = self.monitor()
        m.pending = [{"impulse_ns": float("nan"), "separation_m": 0.}]
        with self.assertRaises(InvalidSimulationState):
            m.after_step(.001)

    def retreat_monitor(self):
        monitor = self.monitor()
        contact = {"robot_collider": "/Robot/link6/Camera", "other_collider": "/Box",
                   "impulse_ns": .005, "separation_m": .0003}
        monitor.pending = [dict(contact)]
        monitor.after_step(.001)
        monitor.begin_action(allow_retreat=True)
        return monitor, contact

    def test_retreat_can_leave_existing_contact_without_waiving_new_contacts(self):
        monitor, contact = self.retreat_monitor()
        monitor.pending = [{**contact, "impulse_ns": .003, "separation_m": .0004}]
        monitor.after_step(.001)
        self.assertIsNone(monitor.blocked)
        monitor.pending = [{**contact, "other_collider": "/OtherBox"}]
        monitor.after_step(.001)
        self.assertIsNotNone(monitor.blocked)

    def test_retreat_stops_if_contact_deepens_or_force_grows(self):
        for change in ({"separation_m": -.0003}, {"impulse_ns": .006}):
            with self.subTest(change=change):
                monitor, contact = self.retreat_monitor()
                monitor.pending = [{**contact, **change}]
                monitor.after_step(.001)
                self.assertIsNotNone(monitor.blocked)

    def test_retreat_cannot_push_on_persistent_contact_or_deep_overlap(self):
        monitor, contact = self.retreat_monitor()
        monitor.after_step(.25)
        monitor.pending = [contact]
        monitor.after_step(.001)
        self.assertIsNotNone(monitor.blocked)
        monitor.blocked["separation_m"] = -.003
        monitor.begin_action(allow_retreat=True)
        monitor.pending = [{**contact, "separation_m": -.0025}]
        monitor.after_step(.001)
        self.assertIsNotNone(monitor.blocked)

    def test_controller_holds_once_and_reports_blocked_without_goal_success(self):
        r = R5Adapter()
        r.targets = {"right": (0.,) * 8}
        row = {"arm": "right", "component": "wrist_camera_or_mount", "force_n": .4}
        r.backend = SimpleNamespace(collision_monitor=SimpleNamespace(blocked=row))
        r.on_action_timeout = Mock()
        self.assertEqual(r.feedback().status, "collision_blocked")
        self.assertEqual(r.feedback().metrics["collision"], {"arm": "right", "component": "wrist_camera_or_mount"})
        self.assertNotIn("force_n", r.feedback().metrics["collision"])
        self.assertEqual(r.collision_stop["force_n"], .4)
        r.on_action_timeout.assert_called_once()

    def test_episode_returns_collision_to_policy_after_one_tick(self):
        import json
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from fixtures import robot_config
        from robot_tasks.backends.mock import MockBackend, MockRobot, MockTask, NoopPolicy
        from robot_tasks.runner import run_episode
        from robot_tasks.types import Action, ArmCommand, Feedback

        config = robot_config()
        observed = []

        class ContactRobot(MockRobot):
            collision_stop = None

            def apply(self, action):
                self.collision_stop = {"force_n": .4, "component": "wrist_camera_or_mount"}

            def feedback(self):
                return Feedback("collision_blocked" if self.collision_stop else "completed")

        class Policy(NoopPolicy):
            def act(self, context):
                observed.append(context.observation)
                if len(observed) == 1:
                    return Action((ArmCommand("left", "gripper", (.035,)),), self.duration_s)
                return Action((), self.duration_s, request_stop=True)

        with TemporaryDirectory() as folder:
            path, result = run_episode(config, MockBackend(), ContactRobot(), MockTask(),
                                       Policy(config.control_dt_s), Path(folder), dry_run=True, step_limit=3)
            events = [json.loads(line) for line in (path / "episode.jsonl").read_text().splitlines()]
        self.assertEqual(len(observed), 2)
        self.assertEqual(observed[1].feedback.status, "collision_blocked")
        self.assertAlmostEqual(observed[1].time_s - observed[0].time_s, config.control_dt_s)
        self.assertEqual(result["action_timeouts"], 0)
        self.assertIsNone(result["success"])
        self.assertTrue(any(e["event"] == "collision_blocked" for e in events))


if __name__ == "__main__":
    unittest.main()
