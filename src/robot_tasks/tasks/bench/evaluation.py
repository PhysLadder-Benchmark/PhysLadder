"""Independent streaming scoring. Model completion claims never enter this module."""

import math

import numpy as np

from ...robots.transforms import pose_transform
from ...types import Assessment


def held(row, arm=None):
    groups = [row["fingers"][arm]] if arm else row["fingers"].values()
    return any(min(fs) > 0.05 for fs in groups)


def released(row):
    return all(max(fs) < 0.05 for fs in row["fingers"].values())


def quiet(row):
    return (
        np.linalg.norm(row["linear_velocity"]) < 0.025
        and np.linalg.norm(row["angular_velocity"]) < 0.3
    )


def upright(row, angle=0.22):
    return pose_transform(row["pose"])[2, 2] >= math.cos(angle)


def table_placed(row):
    return released(row) and quiet(row) and row["bench_force_n"] > 0.05


def relative(row, frame):
    return np.linalg.inv(pose_transform(frame["pose"])) @ pose_transform(row["pose"])


class BenchEvaluator:
    def __init__(self, spec, initial):
        self.spec, self.initial = spec, initial
        self.last_time = self.stable_since = None
        self.grasp_since, self.grasped, self.touched = {}, set(), set()
        self.flags = set()
        self.timers = {}
        self.peak_force = self.max_strain = 0.0
        self.previous_angle = None
        self.push_contact_pose = None
        self.net_turn = 0.0

    def update(self, now, rows):
        if not math.isfinite(now) or (
            self.last_time is not None and now <= self.last_time
        ):
            raise ValueError("Household scoring requires finite increasing time")
        self.last_time = now
        for name, row in rows.items():
            values = [
                *row["pose"],
                *row["linear_velocity"],
                *row["angular_velocity"],
                row["fixture_force_n"],
                row["bench_force_n"],
                *[f for fs in row["fingers"].values() for f in fs],
                *row["pair_forces"].values(),
            ]
            if not np.isfinite(values).all():
                raise ValueError("Nonfinite household measurement")
            self.peak_force = max(
                self.peak_force, *[f for fs in row["fingers"].values() for f in fs]
            )
            if held(row):
                self.touched.add(name)
            if held(row) and row["pose"][2] >= self.initial[name][2] + 0.015:
                self.grasp_since.setdefault(name, now)
                if now - self.grasp_since[name] >= 0.15 - 1e-9:
                    self.grasped.add(name)
            else:
                self.grasp_since.pop(name, None)
        from . import scoring_l1, scoring_l2, scoring_l3

        rules = {**scoring_l1.RULES, **scoring_l2.RULES, **scoring_l3.RULES}
        complete, details, failure = rules[self.spec.design.task_id](self, rows)
        # Reset only on observable task failure; never change objects to manufacture success.
        fallen = [
            n for n, r in rows.items() if np.min(np.asarray(r["corners"])[:, 2]) < -0.08
        ]
        if fallen:
            failure = "Object fell below the table: " + ", ".join(fallen)
        if self.spec.design.task_id in {"L1-01", "L1-03", "L1-05", "L1-08"}:
            for name, row in rows.items():
                key = "toppled_" + name
                if not held(row) and not upright(row, math.pi / 3):
                    self.timers.setdefault(key, now)
                    if now - self.timers[key] >= 0.25 - 1e-9:
                        failure = "Upright task object toppled without a grasp: " + name
                else:
                    self.timers.pop(key, None)
        self.stable_since = (
            (now if self.stable_since is None else self.stable_since)
            if complete and not failure
            else None
        )
        duration = 0 if self.stable_since is None else now - self.stable_since
        metrics = {
            "task_version": self.spec.design.version,
            "mechanism": self.spec.mechanism,
            "limitations": self.spec.limitations,
            "criteria": details,
            "objects": rows,
            "grasp_lift_confirmed": sorted(self.grasped),
            "process_flags": sorted(self.flags),
            "peak_finger_force_n": self.peak_force,
            "stable_duration_s": duration,
            "goal_complete": bool(complete),
            "stage": "stabilize" if complete else "manipulate",
        }
        if failure:
            return Assessment("failed", failure, metrics, retryable=True)
        stability = self.spec.parameters.get("hold_s", 1.0)
        if complete and duration >= stability - 1e-9:
            metrics["stage"] = "complete"
            return Assessment(
                "succeeded",
                "Measured physical goal and process constraints met" if stability == 0
                else "Measured physical goal and process constraints sustained",
                metrics,
            )
        return Assessment("running", "Waiting for measured task completion", metrics)
