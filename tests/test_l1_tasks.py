"""Failure-oriented L1 scoring and fixed experimental interface regressions."""

import math
import unittest
from dataclasses import replace
from unittest.mock import Mock

from robot_tasks.config import ExecutionSettings
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.l1.evaluation import L1Evaluator, L1Settings
from robot_tasks.tasks.l1.specs import RIGID_IDS, build_design, local_corners, pose
from robot_tasks.tasks.l1.task import L1Task
from robot_tasks.types import ArmState, EvaluationState, Feedback


def sample(a, p, *, grip=False, handle=True):
    t = pose_transform(p)
    points = (
        [[[-0.025, 0.010, 0.0]], [[-0.025, -0.010, 0.0]]]
        if handle
        else [[[0.04, 0.0, 0.0]], [[0.04, 0.0, 0.0]]]
    )
    return {
        "pose": list(p),
        "linear_velocity": [0.0, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.0],
        "finger_forces": [1.0, 1.0] if grip else [0.0, 0.0],
        "finger_contacts_local": points if grip else [[], []],
        "fixture_force_n": a.mass * 9.81,
        "bench_force_n": 0.0,
        "corners": (local_corners(a) @ t[:3, :3].T + t[:3, 3]).tolist(),
    }


class L1ScoreTests(unittest.TestCase):
    def setup_task(self, task_id):
        self.d = build_design(task_id)
        self.cfg = L1Settings(position_jitter_m=0, yaw_jitter_rad=0)
        self.e = L1Evaluator(self.d, self.cfg, {a.name: a.initial for a in self.d.articles})
        self.time = -0.25

    def update(self, rows, **kwargs):
        self.time += 0.25
        return self.e.update(self.time, rows, tcp=(0.1, 0.1, 0.4, 1, 0, 0, 0), gripper_width=0.07, **kwargs)

    def grasp(self, handle=True):
        rows = {
            a.name: sample(a, pose(a.initial[0], a.initial[1], 0.22), grip=True, handle=handle)
            for a in self.d.articles
        }
        for _ in range(3):
            self.update(rows)
        if self.d.insertion:
            # A real grasp/lift must precede reaching insertion depth while held.
            self.update({a.name: sample(a, a.goal, grip=True) for a in self.d.articles})
        if self.d.kind == "slot":
            a = self.d.articles[0]
            above = {a.name: sample(a, pose(a.goal[0], a.goal[1], 0.22), grip=True)}
            self.update(above)
            inside = {a.name: sample(a, pose(a.goal[0], a.goal[1], 0.04))}
            self.update(inside)

    def goals(self):
        return {a.name: sample(a, a.goal) for a in self.d.articles}

    def test_all_eight_positive_histories_and_stability_delay(self):
        for task_id in RIGID_IDS:
            with self.subTest(task=task_id):
                self.setup_task(task_id)
                self.grasp()
                self.assertEqual(self.update(self.goals()).status, "running")
                for _ in range(5):
                    result = self.update(self.goals())
                self.assertEqual(result.status, "succeeded")
                self.assertTrue(result.metrics["process_constraints_met"])

    def test_goal_pose_alone_never_succeeds(self):
        for task_id in RIGID_IDS:
            with self.subTest(task=task_id):
                self.setup_task(task_id)
                for _ in range(8):
                    result = self.update(self.goals())
                self.assertNotEqual(result.status, "succeeded")
                self.assertFalse(result.metrics["process_constraints_met"])

    def test_wrong_hole_or_wrong_orientation_rejected(self):
        for task_id in ("L1-01", "L1-02", "L1-03", "L1-05", "L1-08"):
            with self.subTest(task=task_id):
                self.setup_task(task_id)
                self.grasp()
                rows = {
                    a.name: sample(a, pose(a.goal[0] + 0.02, a.goal[1], a.goal[2])) for a in self.d.articles
                }
                for _ in range(8):
                    result = self.update(rows)
                self.assertFalse(result.metrics["goal_complete"])
        self.setup_task("L1-01")
        self.grasp()
        a = self.d.articles[0]
        horizontal = {a.name: sample(a, pose(*a.goal[:3], roll=math.pi / 2))}
        self.assertFalse(self.update(horizontal).metrics["goal_complete"])

    def test_peg_below_socket_floor_is_not_success(self):
        self.setup_task("L1-01")
        self.grasp()
        a = self.d.articles[0]
        rows = {a.name: sample(a, pose(a.goal[0], a.goal[1], self.d.insertion.height_m / 2))}
        for _ in range(8):
            result = self.update(rows)
        self.assertFalse(result.metrics["goal_complete"])
        self.assertNotEqual(result.status, "succeeded")

    def test_wide_insertion_accepts_offset_rotation_and_either_end(self):
        for roll in (0., math.pi):
            with self.subTest(roll=roll):
                self.setup_task("L1-01")
                self.grasp()
                a = self.d.articles[0]
                p = pose(a.goal[0] + .009, a.goal[1], a.goal[2], roll=roll, yaw=2.1)
                self.update({a.name: sample(a, p, grip=True)})
                for _ in range(6):
                    result = self.update({a.name: sample(a, p)})
                self.assertEqual(result.status, "succeeded")
                self.assertEqual(result.metrics["task_version"], "l1-01-wide-clearance-v3")

    def test_lost_peg_interrupts_attempt_even_after_landing_and_before_grasp_hold(self):
        for confirmed in (False, True):
            with self.subTest(confirmed=confirmed):
                self.setup_task("L1-01")
                a = self.d.articles[0]
                for _ in range(3 if confirmed else 1):
                    self.update({a.name: sample(a, pose(*a.initial[:2], .22), grip=True)})
                # Landed between captures: downward velocity is no longer visible.
                rows = {a.name: sample(a, a.initial)}
                self.assertEqual(self.update(rows).status, "running")
                result = self.update(rows)
                self.assertEqual(result.status, "failed")
                self.assertTrue(result.retryable)
                self.assertIn("lost its grasp", result.reason)
                self.assertEqual(result.metrics["stage"], "failed")
                # A later goal pose cannot erase this failed attempt.
                for _ in range(6):
                    result = self.update(self.goals())
                self.assertEqual(result.status, "failed")

    def test_contact_flicker_does_not_reset_a_grasp(self):
        self.setup_task("L1-01")
        a = self.d.articles[0]
        p = pose(*a.initial[:2], .22)
        for _ in range(3):
            self.update({a.name: sample(a, p, grip=True)})
        self.assertEqual(self.update({a.name: sample(a, p)}).status, "running")
        for _ in range(3):
            self.assertEqual(self.update({a.name: sample(a, p, grip=True)}).status, "running")

    def test_toppled_peg_before_first_grasp_is_retryable_but_held_tilt_is_allowed(self):
        for grip in (False, True):
            with self.subTest(grip=grip):
                self.setup_task("L1-01")
                a = self.d.articles[0]
                rows = {a.name: sample(a, pose(*a.initial[:2], .1, roll=math.pi / 2), grip=grip)}
                for _ in range(4):
                    result = self.update(rows)
                self.assertEqual(result.status, "running" if grip else "failed")
                self.assertEqual(result.retryable, not grip)

    def test_guided_release_may_settle_without_being_classified_as_a_drop(self):
        self.setup_task("L1-01")
        self.grasp()
        a = self.d.articles[0]
        p = pose(*a.goal[:2], a.goal[2] + .004)
        self.update({a.name: sample(a, p, grip=True)})
        rows = {a.name: sample(a, p)}
        rows[a.name]["linear_velocity"][2] = -.1
        rows[a.name]["fixture_force_n"] = 0.
        self.assertEqual(self.update(rows).status, "running")
        for _ in range(6):
            result = self.update(self.goals())
        self.assertEqual(result.status, "succeeded")

    def test_below_table_peg_is_retryable_without_waiting_for_contact_debounce(self):
        self.setup_task("L1-01")
        a = self.d.articles[0]
        result = self.update({a.name: sample(a, pose(*a.initial[:2], -.2))})
        self.assertEqual(result.status, "failed")
        self.assertTrue(result.retryable)

    def test_dropped_into_hole_without_guided_insertion_does_not_pass(self):
        self.setup_task("L1-01")
        a = self.d.articles[0]
        for _ in range(3):
            self.update({a.name: sample(a, pose(a.initial[0], a.initial[1], .22), grip=True)})
        for _ in range(6):
            result = self.update(self.goals())
        self.assertTrue(result.metrics["goal_complete"])
        self.assertFalse(result.metrics["process_constraints_met"])
        self.assertFalse(result.metrics["criteria"][a.name]["guided_insertion_confirmed"])

    def test_shallow_insert_or_missing_socket_support_does_not_pass(self):
        for case in ("shallow", "floating", "bench_support"):
            with self.subTest(case=case):
                self.setup_task("L1-01")
                self.grasp()
                a = self.d.articles[0]
                rows = self.goals()
                if case == "shallow":
                    rows[a.name] = sample(a, pose(a.goal[0], a.goal[1], a.goal[2] + .015))
                elif case == "floating":
                    rows[a.name]["fixture_force_n"] = 0.0
                else:
                    rows[a.name]["bench_force_n"] = a.mass * 9.81
                for _ in range(6):
                    result = self.update(rows)
                self.assertFalse(result.metrics["goal_complete"])
                self.assertNotEqual(result.status, "succeeded")

    def test_held_object_cannot_count_as_released(self):
        self.setup_task("L1-02")
        self.grasp()
        for _ in range(8):
            result = self.update({a.name: sample(a, a.goal, grip=True) for a in self.d.articles})
        self.assertEqual(result.status, "running")
        self.assertTrue(result.metrics["goal_complete"])

    def test_unstable_goal_restarts_hold_timer(self):
        self.setup_task("L1-08")
        self.grasp()
        for _ in range(3):
            self.update(self.goals())
        rows = self.goals()
        rows["top_block"]["linear_velocity"][0] = 0.1
        self.update(rows)
        self.assertEqual(self.update(self.goals()).metrics["stable_duration_s"], 0.0)

    def test_tool_head_grasp_is_constraint_violation(self):
        self.setup_task("L1-06")
        self.grasp(handle=False)
        for _ in range(8):
            result = self.update(self.goals())
        self.assertTrue(result.metrics["bad_handle_contact"])
        self.assertFalse(result.metrics["process_constraints_met"])

    def test_ring_requires_hook_load_not_pose_only(self):
        self.setup_task("L1-07")
        self.grasp()
        rows = self.goals()
        rows["ring"]["fixture_force_n"] = 0.0
        for _ in range(8):
            result = self.update(rows)
        self.assertFalse(result.metrics["goal_complete"])

    def test_tool_can_recover_with_a_new_valid_handle_grasp(self):
        self.setup_task("L1-06")
        self.grasp(handle=False)
        self.grasp(handle=True)
        for _ in range(8):
            result = self.update(self.goals())
        self.assertTrue(result.metrics["bad_handle_contact"])
        self.assertEqual(result.status, "succeeded")

    def test_slot_requires_crossing_slit_and_rejects_bypass(self):
        self.setup_task("L1-04")
        a = self.d.articles[0]
        for _ in range(3):
            self.update({a.name: sample(a, pose(0.36, 0.25, 0.22), grip=True)})
        # Entire object crosses outside the slit between samples.
        self.update({a.name: sample(a, pose(0.40, 0.25, 0.04))})
        for _ in range(8):
            result = self.update(self.goals())
        self.assertFalse(result.metrics["goal_complete"])

    def test_recovery_after_failed_placement_is_allowed(self):
        self.setup_task("L1-03")
        self.grasp()
        a = self.d.articles[0]
        for _ in range(3):
            self.update({a.name: sample(a, a.initial)})
        for _ in range(6):
            result = self.update(self.goals())
        self.assertEqual(result.status, "succeeded")

    def test_inactive_arm_and_nonfinite_data_fail_closed(self):
        self.setup_task("L1-02")
        self.assertEqual(self.update(self.goals(), inactive_drift=0.02).status, "failed")
        rows = self.goals()
        next(iter(rows.values()))["pose"][0] = float("nan")
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            self.update(rows)

    def test_nonfinite_time_and_robot_state_are_infrastructure_errors(self):
        for field in ("now", "tcp", "gripper_width", "inactive_drift"):
            with self.subTest(field=field):
                self.setup_task("L1-02")
                values = {"now": 0.0, "tcp": [0.1, 0.1, 0.4, 1, 0, 0, 0],
                          "gripper_width": 0.07, "inactive_drift": 0.0}
                values[field] = [float("nan")] * 7 if field == "tcp" else float("nan")
                with self.assertRaisesRegex(ValueError, "Nonfinite"):
                    self.e.update(objects=self.goals(), **values)


class L1ProtocolTests(unittest.TestCase):
    def test_uniform_cylinder_has_gripper_margin_and_real_socket_clearance(self):
        d = build_design("L1-01")
        g = d.insertion
        _, config = resolve_task("L1-01")
        self.assertFalse(d.fixture_asset)
        self.assertFalse(d.articles[0].asset)
        self.assertAlmostEqual(g.hole_radius_m - g.radius_m, .010)
        self.assertGreaterEqual(config.robot.gripper.width_limits_m[1] - 2 * g.radius_m, .030)
        self.assertLessEqual(g.height_m / (2 * g.radius_m), 2.0)
        cylinder, = d.articles[0].parts
        self.assertEqual(cylinder.shape, "cylinder")
        self.assertEqual(cylinder.size, (g.radius_m, g.radius_m, g.height_m))
        self.assertEqual(cylinder.position, (0, 0, 0))
        # The tangent inner faces retain a real opening; no solid central collider.
        for part in d.fixtures[1:]:
            self.assertAlmostEqual(math.dist(part.position[:2], d.articles[0].goal[:2]) - part.size[0] / 2,
                                   g.hole_radius_m)
        self.assertEqual(d.fixture_friction, 0.0)
        self.assertFalse(g.measure(d.articles[0].initial, d.articles[0].goal[:2], .175)["insertion_geometry_met"])

    def test_block_and_ring_offer_grasps_within_fixed_gripper_limits(self):
        _, config = resolve_task("L1-08")
        lower, upper = config.robot.gripper.width_limits_m
        base = build_design("L1-08").articles[0].parts[0]
        self.assertLess(max(base.size[:2]) + 0.004, upper)
        ring = build_design("L1-07").articles[0]
        # Ring rim can be grasped locally without filling its opening.
        self.assertGreater(ring.parts[0].size[2], lower + 0.002)
        self.assertLess(ring.parts[0].size[2], upper)

    def test_all_l1_tasks_use_shared_workflow_and_observation_arm(self):
        _, reference = resolve_task("GPT-PICK-PLACE")
        from robot_tasks.tasks.bench.task import BenchTask
        for name in ("L1-01", "L1-03", "L1-05", "L1-07", "L1-08", "L1-09"):
            spec, config = resolve_task(name)
            self.assertTrue(spec.implemented)
            self.assertEqual(config.robot.source, reference.robot.source)
            self.assertEqual(config.cameras, reference.cameras)
            for section in ("observation", "robot", "policy", "recording"):
                self.assertEqual(config.raw[section], reference.raw[section], (name, section))
            execution = dict(config.raw["execution"])
            execution.pop("retry_failed_attempts", False)
            self.assertEqual(execution, reference.raw["execution"])
            self.assertNotIn("policy_options", config.raw)
            self.assertEqual(config.observation_arms, ("right",))
            self.assertTrue(config.include_depth)
            self.assertEqual(ExecutionSettings.from_config(config).max_decisions, 100)
            self.assertEqual(config.raw["policy"]["reasoning_effort"], "medium")
            self.assertEqual(config.control_dt_s, reference.control_dt_s)
            self.assertEqual(config.physics_dt_s, reference.physics_dt_s)
            prompt = BenchTask.prepare_config(config).instruction
            self.assertNotIn("metadata", prompt)
            self.assertNotIn("sha256", prompt)
            self.assertIn("observation arm", prompt)
            self.assertNotIn("keep the other arm still", prompt)

    def test_observation_arm_motion_does_not_fail_l1_scoring(self):
        for task_id in RIGID_IDS:
            with self.subTest(task=task_id):
                _, config = resolve_task("L1-01")
                task = L1Task()
                task.arm = config.arms[0]
                task.fixed_arms = config.fixed_arms
                task.initial_robot = None
                task.inactive_drift = 0.0
                task.scene = Mock()
                task.scorer = Mock()
                state = {"left": ArmState(joint_positions_rad=(0.0,) * 6),
                         "right": ArmState(joint_positions_rad=(0.0,) * 6)}
                task.evaluate(EvaluationState({"robot": state, "simulation_time_s": 0.0}), Feedback("completed"))
                state = {**state, "right": ArmState(joint_positions_rad=(0.5,) * 6)}
                task.evaluate(EvaluationState({"robot": state, "simulation_time_s": 1.0}), Feedback("completed"))
                self.assertEqual(task.scorer.update.call_args.kwargs["inactive_drift"], 0.0)

    def test_unknown_settings_and_wrong_active_arm_fail(self):
        _, c = resolve_task("L1-01")
        with self.assertRaises(ValueError):
            L1Task.prepare_config(replace(c, arms=("right",)))
        with self.assertRaises(ValueError):
            L1Settings.from_config(replace(c, raw={**c.raw, "l1": {"hidden_override": 1}}))
        with self.assertRaisesRegex(ValueError, "legacy"):
            L1Task.prepare_config(replace(c, raw={**c.raw, "l1": {"position_tolerance_m": .003}}))


if __name__ == "__main__":
    unittest.main()
