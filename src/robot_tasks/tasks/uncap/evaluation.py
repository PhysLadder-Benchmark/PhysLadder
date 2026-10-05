"""Independent contact/process scoring. Never supplies object truth to GPT."""

import numpy as np

from ...robots.transforms import pose_transform
from ...types import Assessment
from .spec import SPEC, surface


class UncapEvaluator:
    def __init__(self, *, require_table_pickup=False):
        self.require_table_pickup = require_table_pickup
        self.assembly_lifted = False
        self.separated_while_held = False
        self.dual_grasp_seen = False
        self.peak_finger_force = 0.0
        self.peak_pair_force = 0.0
        self.stable_since = None
        self.last_time = None

    def update(self, now, objects, robot):
        if self.last_time is not None and now <= self.last_time:
            raise ValueError("Uncapping samples require increasing simulation time")
        self.last_time = now
        held = {
            name: all(f > SPEC.contact_threshold for f in objects[name]["fingers"][arm])
            for name, arm in (("body", "left"), ("cap", "right"))
        }
        self.dual_grasp_seen |= all(held.values())
        self.peak_finger_force = max(
            self.peak_finger_force,
            max(
                f
                for obj in objects.values()
                for fs in obj["fingers"].values()
                for f in fs
            ),
        )
        self.peak_pair_force = max(self.peak_pair_force, objects["cap"]["pair_force_n"])
        transforms = {
            name: pose_transform(obj["pose"]) for name, obj in objects.items()
        }
        body_tip = transforms["body"] @ np.array([0, 0, SPEC.body_length / 2, 1])
        assembled = np.linalg.inv(transforms["body"]) @ transforms["cap"]
        if (
            held["body"]
            and min(objects[n]["pose"][2] for n in ("body", "cap")) > 0.045
            and abs(assembled[2, 3] - SPEC.cap_offset) < 0.005
            and np.linalg.norm(assembled[:2, 3]) < 0.005
        ):
            self.assembly_lifted = True
        tip_in_cap = np.linalg.inv(transforms["cap"]) @ body_tip
        # The complete body tip must clear the OPEN end, not penetrate the closed cap.
        gap = -SPEC.cap_length / 2 - float(tip_in_cap[2])
        separated = (
            gap >= 0.008 and objects["cap"]["pair_force_n"] < SPEC.contact_threshold
        )
        if separated and all(held.values()) and self.dual_grasp_seen:
            self.separated_while_held = True
        placed = {}
        left_table = False
        for name, obj in objects.items():
            t = transforms[name]
            points = surface(name) @ t[:3, :3].T + t[:3, 3]
            released = all(
                max(fs) < SPEC.contact_threshold for fs in obj["fingers"].values()
            )
            supported = obj["bench_force_n"] > SPEC.contact_threshold
            quiet = (
                np.linalg.norm(obj["linear_velocity"]) < 0.02
                and np.linalg.norm(obj["angular_velocity"]) < 0.2
            )
            placed[name] = bool(released and supported and quiet)
            # A small released descent is part of ordinary placement. Only
            # actually falling below the work surface ends the episode early.
            left_table |= bool(points[:, 2].max() < -0.03 and released)
        force_exceeded = (self.peak_finger_force > SPEC.max_finger_force
                          or self.peak_pair_force > SPEC.max_pair_force)
        metrics = {
            "variant": SPEC.version,
            "pregrasped_body": False if self.require_table_pickup else None,
            "assembly_lifted": self.assembly_lifted,
            "dual_grasp_seen": self.dual_grasp_seen,
            "separated_while_held": self.separated_while_held,
            "current_gap_m": gap,
            "peak_finger_force_n": self.peak_finger_force,
            "peak_pair_force_n": self.peak_pair_force,
            "placed": placed,
            "damage_model": "none; force limits are protocol proxies",
            "scoring_protocol": "recoverable_progress_v1",
            "goal_complete": False,
            "quality_warnings": (["Contact force exceeded uncalibrated engineering reference"]
                                 if force_exceeded else []),
            "force_reference_exceeded": force_exceeded,
            "constraint_violations": ["A part left the table"] if left_table else [],
            "process_constraints_satisfied": not left_table,
        }
        if left_table:
            return Assessment(
                "failed", "A part left the table", metrics
            )
        # No arbitrary spacing between the resting parts. Just exclude putting
        # the cap back onto either end of the barrel after the recorded separation.
        body_in_cap = np.linalg.inv(transforms["cap"]) @ transforms["body"]
        ends = np.array(((0, 0, -SPEC.body_length / 2), (0, 0, SPEC.body_length / 2)))
        ends = ends @ body_in_cap[:3, :3].T + body_in_cap[:3, 3]
        recapped = bool(
            abs(body_in_cap[2, 2]) > 0.95
            and any(
                np.linalg.norm(end[:2]) < SPEC.body_radius
                and -SPEC.cap_length / 2 + 0.002 < end[2] < SPEC.cap_length / 2
                for end in ends
            )
        )
        metrics["recapped"] = recapped
        complete = (
            self.separated_while_held
            and not recapped
            and all(placed.values())
            and (self.assembly_lifted or not self.require_table_pickup)
        )
        metrics["goal_complete"] = bool(complete)
        self.stable_since = (
            (now if self.stable_since is None else self.stable_since)
            if complete
            else None
        )
        if (
            self.stable_since is not None
            and now - self.stable_since >= SPEC.stability_s - 1e-9
        ):
            return Assessment(
                "succeeded",
                "Held separation followed by released stable placement of both parts",
                metrics,
            )
        return Assessment(
            "running",
            "Waiting for physical uncapping and separate stable placement",
            metrics,
        )
