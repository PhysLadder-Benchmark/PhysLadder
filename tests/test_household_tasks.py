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
        if spec.design.task_id == "L2-04" and a.name.startswith("load_"):
            # The bottom layer rests on the bowl floor, not the tabletop.
            result[a.name]["bench_force_n"] = 0.0
            result[a.name]["pair_forces"]["bowl"] = a.mass * 9.81
    return result


class HouseholdTests(unittest.TestCase):
    def test_all_maintained_tasks_have_launchable_bindings_and_roles(self):
        tasks = [
            s for s in TASKS.values() if s.task_id.startswith("L") and s.implemented
        ]
        self.assertEqual(len(tasks), 17)
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

    def test_drawer_requires_grasped_opening_then_release_while_open(self):
        spec = build_spec("L2-07")
        rows = measurement(spec)
        initial = {a.name: a.initial for a in spec.design.articles}
        e = BenchEvaluator(spec, initial)
        for i in range(1, 20):
            self.assertEqual(e.update(i * 0.1, rows).status, "running")
        rows["drawer"]["pose"][0] -= 0.11
        rows["drawer"]["fingers"]["left"] = [1, 1]
        self.assertEqual(e.update(2, rows).status, "running")
        rows["drawer"]["fingers"]["left"] = [0, 0]
        for i in range(21, 35):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        rows["drawer"]["pose"][0] += 0.11
        self.assertEqual(e.update(3.6, rows).status, "running")
        fresh = BenchEvaluator(spec, initial)
        rows["drawer"]["pose"][0] -= 0.11
        self.assertEqual(fresh.update(0.1, rows).status, "running")

    def test_books_only_require_single_arm_complete_extraction(self):
        spec = build_spec("L2-03")
        self.assertEqual([a.name for a in spec.design.articles], ["middle_book"])
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        row = rows["middle_book"]
        row["pose"][0] -= 0.10
        row["corners"] = (np.asarray(row["corners"]) - [0.10, 0, 0]).tolist()
        self.assertEqual(e.update(0.1, rows).status, "running")
        row["fingers"]["left"] = [1, 1]
        for i in range(2, 16):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertFalse(result.metrics["criteria"]["release_required"])
        row["fingers"]["left"] = [0, 0]
        self.assertEqual(e.update(1.7, rows).status, "running")

    def test_preloaded_bowl_requires_lift_transport_and_released_placement(self):
        spec = build_spec("L2-04")
        rows = measurement(spec)
        initial = {a.name: a.initial for a in spec.design.articles}
        e = BenchEvaluator(spec, initial)
        for i in range(1, 15):
            self.assertEqual(e.update(i * 0.1, rows).status, "running")
        for row in rows.values():
            row["pose"][2] += 0.04
            row["corners"] = (np.asarray(row["corners"]) + [0, 0, 0.04]).tolist()
            row["bench_force_n"] = 0
        rows["bowl"]["fingers"] = {"left": [1, 1], "right": [1, 1]}
        e.update(1.5, rows)
        self.assertIn("loaded_bowl_lifted", e.flags)
        for row in rows.values():
            row["pose"][1] -= 0.24
            row["pose"][2] -= 0.04
        rows["bowl"]["bench_force_n"] = 3
        rows["bowl"]["fingers"] = {"left": [0, 0], "right": [0, 0]}
        for i in range(16, 30):
            result = e.update(i * 0.1, rows)
        self.assertEqual(result.status, "succeeded")
        fresh = BenchEvaluator(spec, initial)
        self.assertEqual(fresh.update(0.1, rows).status, "running")
        rows["load_0"]["bench_force_n"] = 0.039
        self.assertEqual(e.update(3.0, rows).status, "failed")
        rows["load_0"]["bench_force_n"] = 0
        self.assertEqual(e.update(3.1, rows).status, "failed")

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

    def test_hanging_mug_succeeds_on_release_without_waiting_for_sway(self):
        spec = build_spec("L1-07")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        # Zero waiting time must not make the initial, unfinished scene pass.
        self.assertEqual(e.update(.1, rows).status, "running")
        r = rows["mug"]
        r["pose"][:3] = [.317, .21, .163]
        r["corners"] = [[.31, .20, .16]]
        r["fixture_force_n"], r["bench_force_n"] = .6, 0.
        r["linear_velocity"], r["angular_velocity"] = [.03, 0, 0], [0, 0, 1.]
        r["fingers"]["left"] = [1, 1]
        self.assertEqual(e.update(.2, rows).status, "running")
        self.assertEqual(e.update(.4, rows).status, "running")
        self.assertIn("mug", e.grasped)
        r["fingers"]["left"] = [0, 0]
        result = e.update(.5, rows)
        self.assertEqual(result.status, "succeeded")
        self.assertEqual(result.metrics["stable_duration_s"], 0.)
        # A released cup with neither rack support nor grasp history cannot pass.
        r["fixture_force_n"] = 0.
        self.assertEqual(e.update(.6, rows).status, "running")
        r["fixture_force_n"] = .6
        fresh = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        self.assertEqual(fresh.update(.1, rows).status, "running")

    def test_spinning_cap_in_air_cannot_count_as_unscrewing(self):
        import math

        spec = build_spec("L3-03")
        rows = measurement(spec)
        e = BenchEvaluator(spec, {a.name: a.initial for a in spec.design.articles})
        rows["bottle"]["fingers"]["left"] = [1, 1]
        rows["cap"]["fingers"]["right"] = [1, 1]
        rows["cap"]["pose"][2] = (
            rows["bottle"]["pose"][2] + spec.parameters["engaged_cap_z_max"] + 0.10
        )
        for i in range(1, 30):
            angle = i * 0.4
            rows["cap"]["pose"][3:] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
            e.update(i * 0.1, rows)
        self.assertEqual(e.net_turn, 0)
        self.assertNotIn("threaded_disengagement", e.flags)


if __name__ == "__main__":
    unittest.main()
