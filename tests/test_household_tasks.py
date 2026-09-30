"""Offline geometry/process regressions; no Isaac startup or live GPT calls."""

import json
import unittest
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path

import numpy as np

from robot_tasks.registry import TASKS, resolve_task
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.bench import l1, l2, l3
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.l1.specs import local_corners

IDS = tuple(l1.BUILDERS | l2.BUILDERS | l3.BUILDERS)
ROOT = Path(__file__).resolve().parents[1]


def measurement(spec):
    result = {}
    for a in spec.design.articles:
        t = pose_transform(a.initial)
        corners = local_corners(a) @ t[:3, :3].T + t[:3, 3]
        result[a.name] = {
            "pose": list(a.initial),
            "corners": corners.tolist(),
            "linear_velocity": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "finger_forces": [0.0, 0.0],
            "fingers": {"left": [0.0, 0.0], "right": [0.0, 0.0]},
            "fixture_force_n": 0.0,
            "bench_force_n": a.mass * 9.81 if corners[:, 2].min() < 0.006 else 0.0,
            "pair_forces": {
                b.name: 0.0 for b in spec.design.articles if b.name != a.name
            },
        }
    return result


class HouseholdTests(unittest.TestCase):
    def test_all_maintained_tasks_have_launchable_bindings_and_roles(self):
        tasks = [
            s for s in TASKS.values() if s.task_id.startswith("L") and s.implemented
        ]
        self.assertEqual(len(tasks), 16)
        for spec in tasks:
            with self.subTest(task=spec.task_id):
                _, cfg = resolve_task(spec.task_id)
                prepared = spec.factory.prepare_config(cfg)
                self.assertEqual(
                    cfg.raw["task"]["launch_command"],
                    f"python tools/launch.py run {spec.task_id} --gui",
                )
                self.assertEqual(cfg.raw["execution"]["max_decisions"], 100)
                self.assertEqual(cfg.raw["policy"]["reasoning_effort"], "medium")
                self.assertEqual(
                    cfg.observation_arms, ("right",) if len(cfg.arms) == 1 else ()
                )
                self.assertTrue(prepared.include_depth)
                self.assertTrue(prepared.raw["task"]["description"])
        with self.assertRaisesRegex(ValueError, "real-only"):
            resolve_task("L3-11")

    def test_every_initial_state_is_incomplete_without_robot_evidence(self):
        for task_id in IDS:
            with self.subTest(task=task_id):
                spec = build_spec(task_id)
                json.dumps(asdict(spec), allow_nan=False)
                rows = measurement(spec)
                evaluator = BenchEvaluator(
                    spec, {a.name: a.initial for a in spec.design.articles}
                )
                for i in range(1, 30):
                    assessment = evaluator.update(i * 0.1, rows)
                    self.assertEqual(assessment.status, "running")
                    json.dumps(asdict(assessment), allow_nan=False)

    def test_marker_requires_real_grasp_lift_and_released_fixture_support(self):
        spec = build_spec("L1-01")
        rows = measurement(spec)
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        rows["marker"]["pose"][:3] = list(spec.design.articles[0].goal[:3])
        rows["marker"]["bench_force_n"] = 0.0
        rows["marker"]["fixture_force_n"] = 0.55
        for i in range(1, 20):
            self.assertNotEqual(evaluator.update(i * 0.1, rows).status, "succeeded")
        rows["marker"]["pose"][:3] = [0.36, 0.42, 0.04]
        rows["marker"]["fingers"]["left"] = [1, 1]
        for i in range(20, 25):
            evaluator.update(i * 0.1, rows)
        rows["marker"]["pose"][:3] = [0.36, 0.22, 0.004]
        rows["marker"]["fingers"]["left"] = [0, 0]
        for i in range(25, 40):
            result = evaluator.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        rows["marker"]["fixture_force_n"] = 0
        self.assertEqual(evaluator.update(4.0, rows).status, "running")

    def test_nested_cups_require_support_and_lift_only_for_upper_cup(self):
        spec = build_spec("L1-08")
        rows = measurement(spec)
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        top, base = rows["upper_cup"], rows["lower_cup"]
        top["pose"][2] = 0.04
        top["fingers"]["left"] = [1, 1]
        for i in range(1, 5):
            evaluator.update(i * 0.1, rows)
        top["pose"][:3] = [0.36, 0.23, 0.005]
        top["fingers"]["left"] = [0, 0]
        top["pair_forces"]["lower_cup"] = 0.45
        base["bench_force_n"] = 1.5
        for i in range(5, 20):
            result = evaluator.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        top["pair_forces"]["lower_cup"] = 0
        self.assertEqual(evaluator.update(2.0, rows).status, "running")

    def test_no_l3_shortcut_by_final_positions_without_process(self):
        for task_id in l3.BUILDERS:
            with self.subTest(task=task_id):
                spec = build_spec(task_id)
                rows = measurement(spec)
                evaluator = BenchEvaluator(
                    spec, {a.name: a.initial for a in spec.design.articles}
                )
                for a in spec.design.articles:
                    rows[a.name]["pose"] = list(a.goal)
                if task_id == "L3-03":
                    rows["cap"]["pose"] = [0.50, 0, 0.03, 1, 0, 0, 0]
                    rows["cap"]["bench_force_n"] = 0.25
                for i in range(1, 30):
                    self.assertNotEqual(
                        evaluator.update(i * 0.1, rows).status, "succeeded"
                    )

    def test_elastic_crushing_proxy_fails_instead_of_reporting_damage_free_success(
        self,
    ):
        spec = build_spec("L2-06")
        rows = measurement(spec)
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        rows["jelly_0"]["pose"][0] -= 0.025
        result = evaluator.update(0.1, rows)
        self.assertEqual(result.status, "failed")
        self.assertIn("strain", result.reason)
        self.assertTrue(result.retryable)

    def test_geometry_and_passive_joints_are_finite_and_self_contained(self):
        for task_id in IDS:
            with self.subTest(task=task_id):
                spec = build_spec(task_id)
                names = {a.name for a in spec.design.articles}
                self.assertEqual(len(names), len(spec.design.articles))
                for a in spec.design.articles:
                    self.assertGreater(a.mass, 0)
                    self.assertTrue(np.isfinite(local_corners(a)).all())
                for joint in spec.joints:
                    self.assertIn(joint.body1, names)
                    self.assertTrue(not joint.body0 or joint.body0 in names)
                    for lower, upper, k, d in joint.axes.values():
                        self.assertLessEqual(lower, upper)
                        self.assertGreaterEqual(k, 0)
                        self.assertGreaterEqual(d, 0)

    def test_scoring_rejects_nonfinite_data_and_stale_time(self):
        spec = build_spec("L1-01")
        rows = measurement(spec)
        evaluator = BenchEvaluator(
            spec, {a.name: a.initial for a in spec.design.articles}
        )
        evaluator.update(0.1, rows)
        with self.assertRaises(ValueError):
            evaluator.update(0.1, rows)
        bad = deepcopy(rows)
        bad["marker"]["pose"][0] = float("nan")
        with self.assertRaises(ValueError):
            evaluator.update(0.2, bad)

    def test_drawer_must_be_opened_while_held_before_closed_counts(self):
        spec = build_spec("L2-07")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        for i in range(1, 20):
            self.assertEqual(e.update(i * 0.1, rows).status, "running")
        rows["drawer"]["pose"][0] += 0.11
        rows["drawer"]["fingers"]["left"] = [1, 1]
        e.update(2, rows)
        rows["drawer"]["pose"][0] -= 0.11
        rows["drawer"]["fingers"]["left"] = [0, 0]
        for i in range(21, 35):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")

    def test_books_require_supported_extraction_before_table_placement(self):
        spec = build_spec("L2-03")
        rows = measurement(spec)
        initial = {a.name: a.initial for a in spec.design.articles}
        e = BenchEvaluator(spec, initial)
        rows["upper_book"]["bench_force_n"] = 0.9
        self.assertEqual(e.update(0.1, rows).status, "failed")
        rows = measurement(spec)
        e = BenchEvaluator(spec, initial)
        rows["middle_book"]["pose"][0] += 0.16
        for n in ("upper_book", "middle_book"):
            rows[n]["fingers"]["left" if n == "upper_book" else "right"] = [1, 1]
        e.update(0.1, rows)
        for n in ("upper_book", "middle_book"):
            rows[n]["fingers"] = {"left": [0, 0], "right": [0, 0]}
            rows[n]["bench_force_n"] = 0.9
            rows[n]["pose"][2] = 0.0125
        for i in range(2, 20):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")

    def test_bowl_requires_loaded_hold_then_released_placement(self):
        spec = build_spec("L2-04")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        bowl = rows["bowl"]
        bowl["pose"][:3] = [0.40, 0, 0.13]
        bowl["fingers"] = {"left": [1, 1], "right": [1, 1]}
        bowl["bench_force_n"] = 0
        rows["feed_gate"]["pose"][0] += 0.12
        for i in range(6):
            rows[f"load_{i}"]["pose"][:3] = [
                0.38 + (i // 2) * 0.02,
                (i % 2) * 0.02 - 0.01,
                0.145,
            ]
        for i in range(1, 26):
            self.assertEqual(e.update(i * 0.1, rows).status, "running")
        self.assertIn("loaded_hold_completed", e.flags)
        bowl["pose"][2] = 0
        bowl["bench_force_n"] = 2
        bowl["fingers"] = {"left": [0, 0], "right": [0, 0]}
        for i in range(6):
            rows[f"load_{i}"]["pose"][2] -= 0.13
        for i in range(26, 40):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        fresh = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        self.assertEqual(fresh.update(0.1, rows).status, "running")

    def test_toppled_marker_retries_but_held_tilt_does_not(self):
        spec = build_spec("L1-01")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        rows["marker"]["pose"][3:] = [2**-0.5, 0, 2**-0.5, 0]
        rows["marker"]["fingers"]["left"] = [1, 1]
        self.assertEqual(e.update(0.1, rows).status, "running")
        self.assertEqual(e.update(0.5, rows).status, "running")
        rows["marker"]["fingers"]["left"] = [0, 0]
        self.assertEqual(e.update(0.6, rows).status, "running")
        result = e.update(0.9, rows)
        self.assertEqual(result.status, "failed")
        self.assertTrue(result.retryable)

    def test_mug_handle_tangency_is_valid_support_not_a_miss(self):
        from robot_tasks.tasks.bench.scoring_l1 import mug

        spec = build_spec("L1-07")
        rows = measurement(spec)
        r = rows["mug"]
        # Hook centre is tangent to the right inner handle wall, with its 4-mm radius.
        r["pose"][:3] = [0.317, 0.21, 0.163]
        r["fixture_force_n"] = 0.6
        r["bench_force_n"] = 0
        r["corners"] = [[0.31, 0.20, 0.16]]
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        _, criteria, _ = mug(e, rows)
        self.assertTrue(criteria["hook_through_handle"])
        self.assertTrue(criteria["hanging_support"])
        r["pose"][0] -= 0.006
        self.assertFalse(mug(e, rows)[1]["hook_through_handle"])

    def test_spinning_cap_in_air_cannot_count_as_unscrewing(self):
        import math

        spec = build_spec("L3-03")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        rows["bottle"]["fingers"]["left"] = [1, 1]
        rows["cap"]["fingers"]["right"] = [1, 1]
        rows["cap"]["pose"][2] = 0.25
        for i in range(1, 30):
            angle = i * 0.4
            rows["cap"]["pose"][3:] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
            e.update(i * 0.1, rows)
        self.assertEqual(e.net_turn, 0)
        self.assertNotIn("threaded_disengagement", e.flags)


if __name__ == "__main__":
    unittest.main()
