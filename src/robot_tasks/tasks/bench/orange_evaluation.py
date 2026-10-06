"""Soft transfer scoring from measured FEM shape, with explicit proxy limits."""

import math

import numpy as np

from ...types import Assessment, InvalidSimulationState


def deformation_metrics(rest_inverse, current, tetrahedra):
    """Principal stretch is invariant to rigid translation/rotation."""
    p = current[tetrahedra]
    edges = np.stack([p[:, i] - p[:, 0] for i in (1, 2, 3)], axis=2)
    gradient = edges @ rest_inverse
    stretches = np.linalg.svd(gradient, compute_uv=False)
    strain = np.max(np.abs(stretches - 1.0), axis=1)
    return {
        "strain_p95": float(np.quantile(strain, 0.95)),
        "strain_max": float(strain.max()),
        "minimum_volume_ratio": float(np.linalg.det(gradient).min()),
    }


class OrangeEvaluator:
    def __init__(self, spec):
        self.spec = spec
        self.last_time = self.stable_since = self.overstrain_since = None
        self.lifted = self.excessive_deformation = False
        self.peak_strain = 0.0

    def update(self, now, rows):
        if not math.isfinite(now) or (
            self.last_time is not None and now <= self.last_time
        ):
            raise InvalidSimulationState("Soft scoring requires increasing finite time")
        self.last_time = now
        row, p = rows["orange"], self.spec.parameters
        low, high = np.asarray(row["bounds_min"]), np.asarray(row["bounds_max"])
        checks = [
            *low,
            *high,
            row["speed_m_s"],
            row["strain_p95"],
            row["strain_max"],
            row["minimum_volume_ratio"],
            *row["finger_distances_m"],
        ]
        if not np.isfinite(checks).all():
            raise InvalidSimulationState("Nonfinite FEM measurement")
        near = row["finger_distances_m"]
        if low[2] > p["table_z"] + p["lift_m"] and all(d < 0.003 for d in near[:2]):
            self.lifted = True
        strain = row["strain_p95"]
        self.peak_strain = max(self.peak_strain, strain)
        if strain > p["strain_limit"]:
            self.overstrain_since = (
                now if self.overstrain_since is None else self.overstrain_since
            )
            if now - self.overstrain_since >= p["deformation_persistence_s"] - 1e-9:
                self.excessive_deformation = True
        else:
            self.overstrain_since = None
        within = bool(
            np.all(low[:2] >= np.asarray(p["target_xy"]) - p["target_half"])
            and np.all(high[:2] <= np.asarray(p["target_xy"]) + p["target_half"])
        )
        supported = bool(abs(low[2] - p["table_z"]) <= p["support_height_tolerance_m"])
        released = all(d > p["finger_clearance_m"] for d in near)
        quiet = row["speed_m_s"] < 0.02
        complete = self.lifted and within and supported and released and quiet
        self.stable_since = (
            (now if self.stable_since is None else self.stable_since)
            if complete
            else None
        )
        duration = 0.0 if self.stable_since is None else now - self.stable_since
        failure = ""
        if high[2] < p["table_z"] - 0.08:
            failure = "Orange fell below the table"
        invalid = row["minimum_volume_ratio"] <= 0
        metrics = {
            "task_version": self.spec.design.version,
            "scoring_protocol": "native_l2_validity_v2",
            "mechanism": self.spec.mechanism,
            "limitations": self.spec.limitations,
            "objects": rows,
            "criteria": {
                "lifted_near_left_fingers": self.lifted,
                "within_target": within,
                "table_supported_geometry": supported,
                "released_geometry": released,
                "quiet": quiet,
                "strain_p95": strain,
                "peak_strain_p95": self.peak_strain,
                "strain_limit": p["strain_limit"],
                "excessive_deformation": self.excessive_deformation,
                "force_measurement_available": False,
            },
            "stages": {
                "lifted": self.lifted,
                "within_target": within,
                "released_on_table": self.lifted and supported and released,
                "stable_placement": complete and duration >= p["hold_s"] - 1e-9,
            },
            "stable_duration_s": duration,
            "goal_complete": complete,
            "process_constraints_satisfied": not (
                failure or self.excessive_deformation
            ),
        }
        if invalid:
            metrics["invalid_reason"] = "simulation_invalid"
            return Assessment(
                "invalid",
                "Inverted FEM elements; numerical validity lost, not modeled tissue damage",
                metrics,
            )
        if failure or self.excessive_deformation:
            metrics["failure_category"] = (
                "deformation_reference_exceeded"
                if self.excessive_deformation
                else "task_failure"
            )
            return Assessment(
                "failed",
                failure or "Declared deformation reference exceeded for 0.25 s",
                metrics,
            )
        if complete and duration >= p["hold_s"] - 1e-9:
            return Assessment(
                "succeeded",
                "Soft segment lifted, transported and released on target",
                metrics,
            )
        return Assessment(
            "running",
            "Waiting for lifted transfer and released tabletop placement",
            metrics,
        )
