"""Independent process scoring: turning the wrist in air cannot earn insertion."""

import math
from dataclasses import dataclass

import numpy as np

from ...robots.transforms import pose_transform
from ...types import Assessment
from .spec import SPEC

SCORE_VERSION = "net-engagement-v2"


@dataclass
class _ThreadProgress:
    """Signed motion within one engagement, with no per-frame credit clipping.

    Controlled motion requires bilateral grip and socket contact at both ends
    of a sample interval. Regrasping pauses this ledger. Physical net motion
    also includes unheld intervals, so a rollback cannot retain stale credit.
    Rotation and descent may settle in different samples: pair their NET totals
    using the thread lead, rather than summing per-sample minima.
    """

    previous: tuple[float, float, bool] | None = None
    held_turn: float = 0.0
    held_advance: float = 0.0
    net_turn: float = 0.0
    net_advance: float = 0.0

    def clear(self):
        self.previous = None
        self.held_turn = self.held_advance = 0.0
        self.net_turn = self.net_advance = 0.0

    def update(self, yaw, z, *, engaged, controlled):
        if not engaged:
            self.clear()
            return
        if self.previous is not None:
            last_yaw, last_z, was_controlled = self.previous
            turn = -math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            advance = last_z - z
            if abs(turn) >= math.pi / 2:
                # Too large a jump to establish continuous rotation reliably.
                self.clear()
            else:
                self.net_turn += turn
                self.net_advance += advance
                if controlled and was_controlled:
                    self.held_turn += turn
                    self.held_advance += advance
        # Do not bank free-air rotation or motion before the first real grasp.
        if controlled or self.previous is not None:
            self.previous = (yaw, z, controlled)

    @property
    def turn(self):
        return max(0.0, min(self.held_turn, self.net_turn))

    @property
    def advance(self):
        return max(
            0.0,
            min(
                self.held_advance,
                self.net_advance,
                SPEC.pitch * self.turn / (2 * math.pi),
            ),
        )


class ScrewEvaluator:
    def __init__(self):
        self.last_time = None
        self.pickup_seen = False
        self.progress = _ThreadProgress()
        self.thread_turn = 0.0
        self.thread_advance = 0.0
        self.guided_depth_seen = False
        self.stable_since = None
        self.unsupported_since = None

    def update(self, now, row):
        numbers = [
            now,
            *row["pose"],
            *row["linear_velocity"],
            *row["angular_velocity"],
            *row["finger_forces"],
            row["fixture_force_n"],
            row["bench_force_n"],
        ]
        if not np.isfinite(numbers).all():
            raise ValueError("Nonfinite screw measurement")
        if self.last_time is not None and now <= self.last_time:
            raise ValueError("Screw samples require increasing simulation time")
        self.last_time = now
        s = SPEC
        t = pose_transform(row["pose"])
        tip, axis = t[:3, 3], t[:3, 2]
        tilt = math.acos(float(np.clip(axis[2], -1, 1)))
        radial_error = float(np.linalg.norm(tip[:2] - s.socket_xy))
        yaw = math.atan2(t[1, 0], t[0, 0])
        depth = s.socket_rim - tip[2]
        held = all(f > s.contact_threshold for f in row["finger_forces"])
        released = max(row["finger_forces"]) < s.contact_threshold
        support = row["fixture_force_n"] > s.contact_threshold
        if held and tip[2] >= s.initial_pose[2] + 0.015:
            self.pickup_seen = True
        aligned = radial_error <= s.alignment_tolerance and tilt <= s.tilt_tolerance_rad
        engaged = aligned and 0.004 <= depth <= s.socket_rim - s.socket_floor + 0.0005
        valid = engaged and held and support and self.pickup_seen
        self.progress.update(yaw, float(tip[2]), engaged=engaged, controlled=valid)
        self.thread_turn = self.progress.turn
        self.thread_advance = self.progress.advance
        physical_goal = (
            engaged and depth >= s.required_depth and tip[2] >= s.socket_floor - 0.0005
        )
        progress_sufficient = (
            self.thread_turn >= s.required_turn_rad
            and self.thread_advance >= s.required_threaded_advance - 0.0005
        )
        # Release can preserve the witness; unwinding/withdrawing cannot.
        if not physical_goal or not progress_sufficient:
            self.guided_depth_seen = False
        elif valid:
            self.guided_depth_seen = True
        quiet = (
            np.linalg.norm(row["linear_velocity"]) < 0.01
            and np.linalg.norm(row["angular_velocity"]) < 0.2
        )
        complete = (
            self.guided_depth_seen and physical_goal and released and support and quiet
        )
        self.stable_since = (
            (now if self.stable_since is None else self.stable_since)
            if complete
            else None
        )
        unsupported = (
            self.pickup_seen
            and released
            and not support
            and row["bench_force_n"] < s.contact_threshold
            and row["linear_velocity"][2] < -0.1
        )
        self.unsupported_since = (
            (now if self.unsupported_since is None else self.unsupported_since)
            if unsupported
            else None
        )
        metrics = {
            "task_version": s.version,
            "score_version": SCORE_VERSION,
            "pickup_seen": self.pickup_seen,
            "insertion_depth_m": float(depth),
            "required_depth_m": s.required_depth,
            "required_turn_rad": s.required_turn_rad,
            "required_threaded_advance_m": s.required_threaded_advance,
            "axis_error_m": radial_error,
            "tilt_rad": tilt,
            "held_thread_turn_rad": self.thread_turn,
            "held_thread_advance_m": self.thread_advance,
            "guided_depth_seen": self.guided_depth_seen,
            "released": released,
            "socket_supported": support,
            "stable_duration_s": 0.0
            if self.stable_since is None
            else now - self.stable_since,
            "damage_model": "none; no stripping or material damage claim",
            # Recorder-only task truth, never part of the policy observation.
            # Store independent values so later samples cannot mutate this row.
            "measurement_time_s": float(now),
            "object_measurement": {
                **{
                    key: [float(value) for value in row[key]]
                    for key in (
                        "pose",
                        "linear_velocity",
                        "angular_velocity",
                        "finger_forces",
                    )
                },
                "fixture_force_n": float(row["fixture_force_n"]),
                "bench_force_n": float(row["bench_force_n"]),
            },
        }
        if (
            tip[2] < -0.03
            or (
                self.unsupported_since is not None
                and now - self.unsupported_since >= 0.25
            )
            or (tilt > math.pi / 3 and released and not physical_goal)
        ):
            return Assessment(
                "failed",
                "Screw dropped or toppled outside the socket",
                metrics,
                retryable=True,
            )
        if (
            self.stable_since is not None
            and now - self.stable_since >= s.stability_s - 1e-9
        ):
            return Assessment(
                "succeeded",
                "Grasped, advanced along the thread and released seated",
                metrics,
            )
        return Assessment(
            "running", "Waiting for held screw advance and released seating", metrics
        )
