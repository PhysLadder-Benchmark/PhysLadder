"""Offline screw geometry and scoring contracts; no simulator/model calls."""

import math
import unittest
from dataclasses import replace

import numpy as np

from robot_tasks.registry import resolve_task
from robot_tasks.tasks.screw.evaluation import ScrewEvaluator
from robot_tasks.tasks.screw.geometry import helix_pieces, socket_shell_mesh
from robot_tasks.tasks.screw.spec import SPEC
from robot_tasks.tasks.screw.task import ScrewTask


ENGAGED_Z = SPEC.socket_rim - 0.010


def sample(z=ENGAGED_Z, yaw=0.0, *, held=True, support=True, xy=None):
    return {
        "pose": [
            *(SPEC.socket_xy if xy is None else xy),
            z,
            math.cos(yaw / 2),
            0,
            0,
            math.sin(yaw / 2),
        ],
        "linear_velocity": [0, 0, 0],
        "angular_velocity": [0, 0, 0],
        "finger_forces": [5, 5] if held else [0, 0],
        "fixture_force_n": 0.9 if support else 0,
        "bench_force_n": 0,
    }


class ScrewTests(unittest.TestCase):
    def setUp(self):
        self.e = ScrewEvaluator()

    def test_shared_workflow_and_observation_arm(self):
        spec, config = resolve_task("L3-05")
        config = spec.factory.prepare_config(config)
        self.assertEqual(config.arms, ("left",))
        self.assertEqual(config.observation_arms, ("right",))
        self.assertEqual(config.raw["execution"]["max_decisions"], 100)
        self.assertEqual(config.raw["policy"]["reasoning_effort"], "medium")
        self.assertEqual(config.grasp_check_mode, "contact_inspection")
        self.assertLess(2 * SPEC.head_radius, config.robot.gripper.width_limits_m[1])
        self.assertGreater(
            np.linalg.norm(np.array(SPEC.source_xy) - SPEC.socket_xy), 0.1
        )
        with self.assertRaises(ValueError):
            ScrewTask.prepare_config(replace(config, arms=("left", "right")))

    def test_thread_pieces_have_closed_outward_surfaces(self):
        for vertices, faces in helix_pieces():
            edges = {}
            for tri in faces:
                for a, b in zip(tri, np.roll(tri, -1)):
                    key = tuple(sorted((int(a), int(b))))
                    edges[key] = edges.get(key, 0) + 1
            self.assertTrue(all(n == 2 for n in edges.values()))
            a, b, c = (vertices[faces[:, i]] for i in range(3))
            volume = np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6
            self.assertGreater(volume, 0)
        pieces = list(helix_pieces())
        np.testing.assert_allclose(
            pieces[32][0][:, :2], pieces[0][0][:, :2], atol=1e-12
        )
        np.testing.assert_allclose(pieces[32][0][:, 2] - pieces[0][0][:, 2], SPEC.pitch)

    def test_socket_has_a_real_open_core(self):
        points, faces = socket_shell_mesh()
        self.assertTrue(np.isfinite(points).all())
        self.assertEqual(faces.shape[1], 3)
        self.assertGreater(
            np.linalg.norm(points[:, :2], axis=1).min(), SPEC.core_radius
        )
        # Continuous surfaces have no duplicated internal section end faces.
        edges = {}
        for tri in faces:
            for a, b in zip(tri, np.roll(tri, -1)):
                key = tuple(sorted((int(a), int(b))))
                edges[key] = edges.get(key, 0) + 1
        self.assertTrue(all(n == 2 for n in edges.values()))

    def test_short_threads_fit_inside_the_shaft_and_socket(self):
        male = np.concatenate([v for v, _ in helix_pieces()])
        female = np.concatenate([v for v, _ in helix_pieces(female=True)])
        self.assertGreater(male[:, 2].min(), 0)
        self.assertLess(male[:, 2].max(), SPEC.shaft_length)
        self.assertGreater(female[:, 2].min(), SPEC.socket_floor)
        self.assertLess(female[:, 2].max(), SPEC.socket_rim)
        self.assertAlmostEqual(SPEC.required_turn_rad / (2 * math.pi), .5)
        self.assertAlmostEqual(SPEC.pitch * SPEC.required_turn_rad / (2 * math.pi),
                               SPEC.required_threaded_advance)
        # The head remains above the rim at the target, not flush with the socket.
        self.assertGreaterEqual(SPEC.shaft_length - SPEC.required_depth, .014)
        self.assertGreater(SPEC.socket_rim - SPEC.required_depth, SPEC.socket_floor)

    def test_half_turn_and_release_pass_but_quarter_turn_does_not(self):
        for steps, success in ((6, False), (12, True)):
            with self.subTest(steps=steps):
                evaluator = ScrewEvaluator()
                evaluator.update(0, sample(0.07, support=False, xy=SPEC.source_xy))
                for i in range(steps + 1):
                    yaw = -i * math.pi / 12
                    z = ENGAGED_Z + SPEC.pitch * yaw / (2 * math.pi)
                    evaluator.update(i + 1, sample(z, yaw))
                released = sample(z, yaw, held=False)
                evaluator.update(steps + 2, released)
                result = evaluator.update(steps + 3, released)
                self.assertEqual(result.status == "succeeded", success)
                self.assertEqual(result.metrics["task_version"], SPEC.version)

    def _turn(self):
        self.e.update(0, sample(0.07, support=False, xy=SPEC.source_xy))
        for i in range(25):
            yaw = -i * math.pi / 12
            z = ENGAGED_Z + SPEC.pitch * yaw / (2 * math.pi)
            result = self.e.update(i + 1, sample(z, yaw))
        self.assertTrue(result.metrics["guided_depth_seen"])
        return z, yaw

    def test_measured_held_turn_then_release_is_success(self):
        z, yaw = self._turn()
        row = sample(z, yaw, held=False)
        self.assertEqual(self.e.update(26, row).status, "running")
        result = self.e.update(27, row)
        self.assertEqual(result.status, "succeeded")
        self.assertGreaterEqual(result.metrics["held_thread_advance_m"], SPEC.required_threaded_advance)

    def test_rotation_and_descent_in_alternating_samples_can_finish(self):
        self.e.update(0, sample(0.07, support=False, xy=SPEC.source_xy))
        z, yaw = ENGAGED_Z, 0.0
        self.e.update(1, sample(z, yaw))
        # Contact settles after rotation, rather than in the same control tick.
        for i in range(12):
            yaw -= math.pi / 6
            self.e.update(2 + 2 * i, sample(z, yaw))
            z -= SPEC.pitch / 12
            result = self.e.update(3 + 2 * i, sample(z, yaw))
        self.assertAlmostEqual(result.metrics["held_thread_turn_rad"], 2 * math.pi)
        self.assertAlmostEqual(result.metrics["held_thread_advance_m"], SPEC.pitch)
        self.e.update(26, sample(z, yaw, held=False))
        self.assertEqual(
            self.e.update(27, sample(z, yaw, held=False)).status, "succeeded"
        )

    def test_zero_net_contact_jitter_does_not_erode_progress(self):
        z, yaw = self._turn()
        for i in range(100):
            self.e.update(26 + 2 * i, sample(z + 0.00001, yaw))
            result = self.e.update(27 + 2 * i, sample(z, yaw))
        self.assertAlmostEqual(result.metrics["held_thread_advance_m"], SPEC.pitch)
        self.assertAlmostEqual(result.metrics["held_thread_turn_rad"], 2 * math.pi)

    def test_back_and_forth_motion_cannot_create_progress(self):
        self.e.update(0, sample(ENGAGED_Z))
        for i in range(100):
            self.e.update(1 + 2 * i, sample(ENGAGED_Z + 0.0005, math.pi / 12))
            result = self.e.update(2 + 2 * i, sample(ENGAGED_Z))
        self.assertAlmostEqual(result.metrics["held_thread_turn_rad"], 0)
        self.assertAlmostEqual(result.metrics["held_thread_advance_m"], 0)
        self.assertFalse(result.metrics["guided_depth_seen"])

    def test_regrasp_preserves_controlled_net_motion(self):
        self.e.update(0, sample(ENGAGED_Z))
        for i in range(1, 13):
            self.e.update(i, sample(ENGAGED_Z - i * 0.0005, -i * math.pi / 12))
        self.e.update(13, sample(ENGAGED_Z - 0.006, -math.pi, held=False))
        self.e.update(14, sample(ENGAGED_Z - 0.006, -math.pi))
        for i in range(1, 13):
            result = self.e.update(
                14 + i, sample(ENGAGED_Z - 0.006 - i * 0.0005, -math.pi - i * math.pi / 12)
            )
        self.assertAlmostEqual(result.metrics["held_thread_advance_m"], SPEC.pitch)
        self.assertTrue(result.metrics["guided_depth_seen"])

    def test_unheld_forward_motion_does_not_earn_controlled_progress(self):
        self.e.update(0, sample(ENGAGED_Z))
        # Only a quarter-turn is held; free motion must not complete the half-turn.
        for i in range(1, 7):
            self.e.update(i, sample(ENGAGED_Z - i * 0.0005, -i * math.pi / 12))
        self.e.update(7, sample(ENGAGED_Z - 0.003, -math.pi / 2, held=False))
        for i in range(1, 19):
            self.e.update(
                7 + i,
                sample(ENGAGED_Z - 0.003 - i * 0.0005, -math.pi / 2 - i * math.pi / 12, held=False),
            )
        result = self.e.update(26, sample(ENGAGED_Z - SPEC.pitch, -2 * math.pi))
        self.assertAlmostEqual(result.metrics["held_thread_advance_m"], 0.003)
        self.assertFalse(result.metrics["guided_depth_seen"])

    def test_rollback_revokes_earned_completion_with_or_without_grip(self):
        for held in (True, False):
            with self.subTest(held=held):
                self.e = ScrewEvaluator()
                z, yaw = self._turn()
                for i in range(1, 25):
                    result = self.e.update(
                        25 + i,
                        sample(z + i * 0.0005, yaw + i * math.pi / 12, held=held),
                    )
                self.assertAlmostEqual(result.metrics["held_thread_advance_m"], 0)
                self.assertFalse(result.metrics["guided_depth_seen"])
                # A later straight push must not resurrect the old success witness.
                self.e.update(50, sample(ENGAGED_Z - SPEC.pitch, held=False))
                self.assertNotEqual(
                    self.e.update(51, sample(ENGAGED_Z - SPEC.pitch, held=False)).status, "succeeded"
                )

    def test_leaving_engagement_resets_credit(self):
        for outside in (sample(SPEC.socket_rim - 0.003), sample(ENGAGED_Z - SPEC.pitch, xy=SPEC.source_xy)):
            with self.subTest(outside=outside):
                self.e = ScrewEvaluator()
                z, yaw = self._turn()
                self.e.update(26, outside)
                result = self.e.update(27, sample(z, yaw))
                self.assertEqual(result.metrics["held_thread_turn_rad"], 0)
                self.assertEqual(result.metrics["held_thread_advance_m"], 0)
                self.assertFalse(result.metrics["guided_depth_seen"])

    def test_unsupported_or_single_finger_motion_does_not_count(self):
        for mode in ("unsupported", "single_finger"):
            with self.subTest(mode=mode):
                evaluator = ScrewEvaluator()
                for i in range(25):
                    row = sample(ENGAGED_Z - i * 0.0005, -i * math.pi / 12)
                    if mode == "unsupported":
                        row["fixture_force_n"] = 0
                    else:
                        row["finger_forces"][1] = 0
                    result = evaluator.update(i, row)
                self.assertEqual(result.metrics["held_thread_advance_m"], 0)
                self.assertFalse(result.metrics["guided_depth_seen"])

    def test_recorded_measurements_are_independent_and_replayable(self):
        import json

        records = []
        for i in range(25):
            row = sample(ENGAGED_Z - i * 0.0005, -i * math.pi / 12)
            records.append(self.e.update(i, row))
            row["pose"][2] = -100  # Reusing an input buffer cannot alter the log.
        for t in (25, 26):
            records.append(self.e.update(t, sample(ENGAGED_Z - SPEC.pitch, -2 * math.pi, held=False)))
        replay = ScrewEvaluator()
        for expected in records:
            metrics = json.loads(json.dumps(expected.metrics, allow_nan=False))
            actual = replay.update(
                metrics["measurement_time_s"], metrics["object_measurement"]
            )
            self.assertEqual(actual, expected)
        self.assertEqual(actual.status, "succeeded")

    def test_runner_stops_when_delayed_motion_and_release_finish(self):
        import contextlib
        import io
        import json
        import tempfile
        from pathlib import Path

        from fixtures import rgbd

        from robot_tasks.backends.mock import MockBackend, MockRobot
        from robot_tasks.runner import run_episode
        from robot_tasks.types import Action

        # Offline recorded-measurement fixture, not a robot control strategy.
        rows = [sample(0.07, support=False, xy=SPEC.source_xy), sample(ENGAGED_Z)]
        z, yaw = ENGAGED_Z, 0.0
        for _ in range(12):
            yaw -= math.pi / 6
            rows.append(sample(z, yaw))
            z -= SPEC.pitch / 12
            rows.append(sample(z, yaw))
        rows.extend(sample(z, yaw, held=False) for _ in range(13))

        class Backend(MockBackend):
            def capture_rgb(self):
                return tuple(rgbd(name, self.t) for name in self.config.cameras)

        class Scene:
            def measure(self):
                return next(self.samples)

            def reset(self, seed):
                self.samples = iter(rows)

        class Task(ScrewTask):
            def build_scene(self, backend, config):
                self.scene = Scene()

        class Policy:
            calls = 0

            def reset(self, seed):
                self.calls = 0

            def act(self, context):
                self.calls += 1
                if self.calls > 1:
                    raise AssertionError(
                        "Task completion must prevent further model calls"
                    )
                if "object_measurement" in context.observation.feedback.metrics:
                    raise AssertionError(
                        "Scoring truth must not enter model observations"
                    )
                return Action((), cfg.control_dt_s)

        _, cfg = resolve_task("L3-05")
        cfg = ScrewTask.prepare_config(cfg)
        cfg = replace(cfg, raw={**cfg.raw, "execution": {"wait_steps": 100}})
        with (
            tempfile.TemporaryDirectory() as directory,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            output, result = run_episode(
                cfg, Backend(), MockRobot(), Task(), Policy(), Path(directory)
            )
            events = [
                json.loads(line)
                for line in (output / "episode.jsonl").read_text().splitlines()
            ]
        self.assertIsNone(result["error"])
        self.assertTrue(result["success"])
        self.assertEqual(result["termination_reason"], "task_terminal")
        self.assertEqual(result["decisions"], 1)
        self.assertEqual(result["rejected_stop_requests"], 0)
        self.assertEqual(result["completed_steps"], len(rows) - 1)
        replay = ScrewEvaluator()
        for event in events:
            if event["event"] == "physics":
                metrics = event["evaluation"]["metrics"]
                score = replay.update(
                    metrics["measurement_time_s"], metrics["object_measurement"]
                )
                self.assertEqual(score.status, event["evaluation"]["status"])
        self.assertEqual(score.status, "succeeded")

    def test_straight_push_cannot_pass(self):
        self.e.update(0, sample(0.07, support=False))
        for i in range(25):
            result = self.e.update(i + 1, sample(SPEC.socket_rim - 0.008 - 0.0005 * i))
        self.assertFalse(result.metrics["guided_depth_seen"])
        self.assertEqual(result.metrics["held_thread_turn_rad"], 0)
        for i in (26, 27, 28):
            self.assertNotEqual(
                self.e.update(i, sample(ENGAGED_Z - SPEC.pitch, held=False)).status, "succeeded"
            )

    def test_free_air_rotation_cannot_pass(self):
        for i in range(25):
            self.e.update(i, sample(0.08, -i * math.pi / 12, support=False))
        for i in (26, 27, 28):
            result = self.e.update(i, sample(ENGAGED_Z - SPEC.pitch, held=False))
            self.assertNotEqual(result.status, "succeeded")
            self.assertEqual(result.metrics["held_thread_turn_rad"], 0)

    def test_already_seated_without_process_is_not_success(self):
        for i in range(5):
            result = self.e.update(i, sample(ENGAGED_Z - SPEC.pitch, held=False))
            self.assertNotEqual(result.status, "succeeded")
        self.assertFalse(result.metrics["pickup_seen"])

    def test_held_final_pose_requires_release(self):
        z, yaw = self._turn()
        for i in (26, 27, 28):
            self.assertEqual(self.e.update(i, sample(z, yaw)).status, "running")

    def test_drop_is_retryable(self):
        result = self.e.update(0, sample(-0.04, held=False, support=False))
        self.assertEqual(result.status, "failed")
        self.assertTrue(result.retryable)

    def test_invalid_samples_fail_explicitly(self):
        self.e.update(1, sample())
        with self.assertRaises(ValueError):
            self.e.update(1, sample())
        with self.assertRaises(ValueError):
            self.e.update(2, sample(float("nan")))


if __name__ == "__main__":
    unittest.main()
