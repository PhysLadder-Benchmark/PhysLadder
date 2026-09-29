"""Shared role permissions and fixed-arm checks, independent of task scoring."""

from dataclasses import replace

from .config import TaskConfig
from .types import Assessment, ArmState


def validate_role_command(config: TaskConfig, arm: str, kind: str) -> None:
    if arm not in config.command_arms:
        raise ValueError(f"Unknown arm: {arm}; no motion permission")
    if arm in config.observation_arms and kind == "gripper":
        raise ValueError(f"Observation arm {arm} cannot receive gripper commands; use it only to move its camera")


class FixedArmMonitor:
    def __init__(self, config: TaskConfig, tolerance_rad: float):
        self.arms, self.tolerance = config.fixed_arms, tolerance_rad
        self.reset()

    def reset(self) -> None:
        self.reference: dict[str, tuple[float, ...]] | None = None
        self.max_drift = 0.0

    def update(self, state: dict[str, ArmState], assessment: Assessment) -> Assessment:
        if self.reference is None:
            self.reference = {a: state[a].joint_positions_rad for a in self.arms if a in state}
        for arm, original in self.reference.items():
            if arm not in state or len(state[arm].joint_positions_rad) != len(original):
                raise ValueError(f"Missing or invalid fixed arm state: {arm}")
            self.max_drift = max(self.max_drift, max(
                (abs(q - q0) for q, q0 in zip(state[arm].joint_positions_rad, original)), default=0.0))
        metrics = {**assessment.metrics, "fixed_arm_max_drift_rad": self.max_drift}
        if self.max_drift > self.tolerance:
            return Assessment("failed", "Fixed arm moved beyond the configured tolerance", metrics)
        return replace(assessment, metrics=metrics)
