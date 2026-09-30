"""Streaming L1 scoring. Only measured simulator state enters this module."""

import math
from dataclasses import dataclass

import numpy as np

from ...robots.transforms import pose_transform
from ...types import Assessment
from .shaped_recess import ShapedRecess


@dataclass(frozen=True)
class L1Settings:
    stability_s: float = 1.0
    grasp_hold_s: float = 0.25
    lift_m: float = 0.015
    position_tolerance_m: float = 0.003
    height_tolerance_m: float = 0.004
    rotation_tolerance_rad: float = 0.14
    speed_m_s: float = 0.02
    angular_speed_rad_s: float = 0.2
    contact_threshold_n: float = 0.05
    retreat_m: float = 0.065
    inactive_joint_tolerance_rad: float = 0.01
    position_jitter_m: float = 0.008
    yaw_jitter_rad: float = 0.15

    @classmethod
    def from_config(cls, config):
        data = dict(config.raw.get("l1", {}))
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"Unknown L1 settings: {sorted(unknown)}")
        result = cls(**data)
        for name, value in vars(result).items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"l1.{name} must be finite and nonnegative")
            if value == 0 and "jitter" not in name:
                raise ValueError(f"l1.{name} must be positive")
        if result.position_jitter_m > 0.02 or result.yaw_jitter_rad > 0.5:
            raise ValueError("L1 v1 jitter exceeds validated reset envelope")
        return result


class InsertionAttemptMonitor:
    """Scoring-only termination rules; never provide object truth to the policy.

    Debounce contact loss and unsupported toppling for 0.25 simulation seconds.
    One missing contact sample is not sufficient evidence of a failed grasp.
    A guided insertion may settle under gravity after release without a reset.
    """

    persistence_s = 0.25
    topple_tilt_rad = math.pi / 3

    def __init__(self):
        self.carried = set()
        self.loss_since = {}
        self.topple_since = {}
        self.failure = ""

    def update(self, name, now, *, lifted, bilateral, tilt, placed):
        if bilateral and lifted:
            self.carried.add(name)
        conditions = (
            (self.loss_since, name in self.carried and not bilateral and not placed,
             "Lifted object lost its grasp before a valid placement"),
            (self.topple_since, tilt >= self.topple_tilt_rad and not bilateral and not placed,
             "Object toppled outside a valid placement"),
        )
        for since, condition, reason in conditions:
            if not condition:
                since.pop(name, None)
            elif now - since.setdefault(name, now) >= self.persistence_s - 1e-9 and not self.failure:
                self.failure = reason


class L1Evaluator:
    def __init__(self, design, settings, initial):
        self.design, self.settings, self.initial = design, settings, initial
        self.last_time = None
        self.grasp_since = {}
        self.grasped = set()
        self.guided_insertions = set()
        self.stable_since = None
        self.previous = {}
        self.above_slot = False
        self.through_slot = False
        self.bad_handle = False
        self.drops = 0
        self.was_dropping = False
        self.insertion_attempt = InsertionAttemptMonitor() if design.insertion else None

    def update(self, now, objects, *, tcp, gripper_width, inactive_drift=0.0):
        controls = [now, inactive_drift]
        if tcp is not None:
            controls.extend(tcp)
        if gripper_width is not None:
            controls.append(gripper_width)
        if not np.isfinite(controls).all():
            raise ValueError("Nonfinite L1 time or robot state")
        if self.last_time is not None and now <= self.last_time:
            raise ValueError("L1 evaluation timestamps must increase")
        self.last_time = now
        cfg = self.settings
        results = {}
        dropping = False
        for a in self.design.articles:
            row = objects[a.name]
            numbers = np.r_[
                row["pose"],
                row["linear_velocity"],
                row["angular_velocity"],
                row["finger_forces"],
                row["fixture_force_n"],
                row["bench_force_n"],
                np.asarray(row["corners"]).ravel(),
            ]
            if not np.isfinite(numbers).all():
                raise ValueError("Nonfinite L1 object state")
            p = np.asarray(row["pose"])
            goal = np.asarray(a.goal)
            r = pose_transform(p)[:3, :3]
            desired = pose_transform(goal)[:3, :3]
            angle = math.acos(float(np.clip((np.trace(desired.T @ r) - 1) / 2, -1, 1)))
            if self.design.insertion:
                # Uniform cylinder: its axis is undirected, so either end may enter.
                angle = math.acos(float(np.clip(abs(r[:, 2] @ desired[:, 2]), 0, 1)))
            if self.design.kind == "stack":
                # Stripes define an undirected axis (180-degree symmetry).
                flip = np.diag([-1.0, -1.0, 1.0])
                angle = min(
                    angle, math.acos(float(np.clip((np.trace((desired @ flip).T @ r) - 1) / 2, -1, 1)))
                )
            contacts = np.asarray(row["finger_forces"])
            bilateral = bool(np.all(contacts >= cfg.contact_threshold_n))
            lifted = p[2] - self.initial[a.name][2] >= cfg.lift_m
            handler = True
            if self.design.kind == "tool":
                # Grip area excludes the head and the neck; both fingers need actual points.
                groups = row["finger_contacts_local"]
                handler = all(
                    len(g) > 0
                    and all(-0.070 <= v[0] <= 0.015 and abs(v[1]) <= 0.014 and abs(v[2]) <= 0.018 for v in g)
                    for g in groups
                )
                if bilateral and not handler:
                    self.bad_handle = True
                    # A bad attempt is logged, but recovery remains possible:
                    # require a new valid handle grasp before completion.
                    self.grasped.discard(a.name)
                # The lifted working end must point along the requested world +X.
                lifted = lifted and angle <= cfg.rotation_tolerance_rad
            grasp = bilateral and lifted and handler
            if grasp:
                self.grasp_since.setdefault(a.name, now)
                if now - self.grasp_since[a.name] >= cfg.grasp_hold_s - 1e-9:
                    self.grasped.add(a.name)
            else:
                self.grasp_since.pop(a.name, None)
            corners = np.asarray(row["corners"])
            local_points = (corners - p[:3]) @ r
            tcp_local = None if tcp is None else (np.asarray(tcp[:3]) - p[:3]) @ r
            distance = (
                0.0
                if tcp_local is None
                else float(
                    np.linalg.norm(
                        tcp_local - np.clip(tcp_local, local_points.min(axis=0), local_points.max(axis=0))
                    )
                )
            )
            released = bool(
                np.max(contacts) < cfg.contact_threshold_n
                and gripper_width is not None
                and gripper_width >= 0.060
                and tcp is not None
                and distance >= cfg.retreat_m
            )
            released_and_retreated = released
            if isinstance(self.design.insertion, ShapedRecess):
                # This shallow-placement task needs actual release, not an
                # additional opening-width or long retreat target.
                released = bool(np.max(contacts) < cfg.contact_threshold_n)
            still = bool(
                np.linalg.norm(row["linear_velocity"]) <= cfg.speed_m_s
                and np.linalg.norm(row["angular_velocity"]) <= cfg.angular_speed_rad_s
            )
            tolerance = cfg.position_tolerance_m if self.design.kind == "insert" else self.design.clearance
            xy = float(np.linalg.norm(p[:2] - goal[:2]))
            z = abs(float(p[2] - goal[2]))
            goal_met = bool(
                xy <= tolerance and z <= cfg.height_tolerance_m and angle <= cfg.rotation_tolerance_rad
            )
            insertion = {}
            if self.design.insertion:
                insertion = self.design.insertion.measure(p, goal[:2], cfg.rotation_tolerance_rad)
                geometry_met = insertion["insertion_geometry_met"]
                if not geometry_met:
                    self.guided_insertions.discard(a.name)
                elif bilateral and a.name in self.grasped:
                    self.guided_insertions.add(a.name)
                supported = bool(row["fixture_force_n"] >= a.mass * 9.81 * 0.5
                                 and row["bench_force_n"] < cfg.contact_threshold_n)
                insertion.update(guided_insertion_confirmed=a.name in self.guided_insertions,
                                 fixture_supported=supported)
                # A held peg can already satisfy geometry. Final success also
                # requires release, so real fixture support becomes mandatory.
                goal_met = bool(geometry_met and (bilateral or supported))
                self.insertion_attempt.update(
                    a.name, now, lifted=lifted, bilateral=bilateral, tilt=angle,
                    placed=bool(geometry_met and a.name in self.guided_insertions),
                )
            if self.design.kind == "slot":
                x, y = goal[:2]
                top = self.design.fixture_top + 0.004
                bottom = self.design.fixture_top - 0.004
                if corners[:, 2].min() > top:
                    self.above_slot = True
                    self.through_slot = False
                previous = self.previous.get(a.name)
                # Test every corner swept across the lid plane, including a complete
                # crossing between two samples; a fast teleport cannot skip the slit.
                valid = True
                crossed = False
                if previous is not None and self.above_slot:
                    for old, new in zip(previous, corners):
                        if old[2] >= top and new[2] < top:
                            fraction = (old[2] - top) / (old[2] - new[2])
                            hit = old + fraction * (new - old)
                            crossed = True
                            valid = valid and abs(hit[0] - x) <= 0.008 and abs(hit[1] - y) <= 0.039
                    if crossed and not valid:
                        self.above_slot = False
                        self.through_slot = False
                    if valid and crossed:
                        self.through_slot = True
                goal_met = bool(
                    self.above_slot
                    and self.through_slot
                    and corners[:, 2].max() < bottom
                    and corners[:, 2].min() >= 0.003
                    and np.max(np.abs(corners[:, :2] - goal[:2])) < 0.060
                )
                self.previous[a.name] = corners.copy()
            if self.design.kind == "ring":
                # Hook horizontal rod must intersect the physical ring aperture.
                centre = np.array([goal[0], goal[1], self.design.fixture_top])
                in_ring = r.T @ (centre - p[:3])
                aperture = bool(np.linalg.norm(in_ring[:2]) < 0.017 and abs(in_ring[2]) < 0.008)
                goal_met = bool(
                    xy <= 0.012
                    and abs(p[2] - goal[2]) < 0.012
                    and aperture
                    and row["fixture_force_n"] >= a.mass * 9.81 * 0.5
                )
            dropping = dropping or bool(
                a.name in self.grasped
                and not bilateral
                and row["linear_velocity"][2] < -0.05
                and not goal_met
            )
            results[a.name] = {
                "goal_complete": goal_met,
                "grasp_lift_confirmed": a.name in self.grasped,
                "released_and_retreated": released_and_retreated,
                "release_complete": released,
                "stable": still,
                "position_error_xy_m": xy,
                "height_error_m": z,
                "rotation_error_rad": angle,
                "bilateral_contact": bilateral,
                "tcp_clearance_m": distance,
                **insertion,
            }
        self.drops += int(dropping and not self.was_dropping)
        self.was_dropping = dropping
        goal_complete = all(r["goal_complete"] for r in results.values())
        process = bool(
            len(self.grasped) == len(results)
            and inactive_drift <= cfg.inactive_joint_tolerance_rad
            and all(r["release_complete"] or not self.design.release for r in results.values())
            and (self.design.insertion is None or len(self.guided_insertions) == len(results))
        )
        qualified = goal_complete and process and all(r["stable"] for r in results.values())
        if qualified:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        duration = 0.0 if self.stable_since is None else now - self.stable_since
        metrics = {
            "goal_complete": goal_complete,
            "process_constraints_met": process,
            "stable_duration_s": duration,
            "objects": objects,
            "criteria": results,
            "drop_events": self.drops,
            "bad_handle_contact": self.bad_handle,
            "inactive_arm_max_drift_rad": inactive_drift,
            "task_version": self.design.version,
        }
        if self.design.insertion:
            metrics["attempt_failure"] = self.insertion_attempt.failure
            metrics["stage"] = (
                "failed" if self.insertion_attempt.failure else
                "verify_grasp" if self.insertion_attempt.loss_since else
                "grasp_and_lift" if not self.grasped else
                "align_and_insert" if not goal_complete or not self.guided_insertions else
                ("release" if isinstance(self.design.insertion, ShapedRecess) else "release_and_retreat") if not process else
                "complete" if duration >= cfg.stability_s - 1e-9 else "stabilize"
            )
        if any(min(v[2] for v in row["corners"]) < -0.10 for row in objects.values()):
            if self.insertion_attempt:
                self.insertion_attempt.failure = "Object fell below table"
                metrics.update(stage="failed", attempt_failure=self.insertion_attempt.failure)
            return Assessment("failed", "Object fell below table", metrics,
                              retryable=self.design.insertion is not None)
        if inactive_drift > cfg.inactive_joint_tolerance_rad:
            return Assessment("failed", "Inactive arm moved", metrics)
        if self.insertion_attempt and self.insertion_attempt.failure:
            return Assessment("failed", self.insertion_attempt.failure, metrics, retryable=True)
        if duration >= cfg.stability_s - 1e-9:
            return Assessment("succeeded", "Physical goal and process constraints sustained", metrics)
        return Assessment("running", "Waiting for goal, grasp evidence and stable completion", metrics)
