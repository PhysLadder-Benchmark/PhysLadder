"""Shared role permissions and fixed-arm checks, independent of task scoring."""

from dataclasses import replace

from .config import TaskConfig
from .types import Assessment, ArmState


ARM_IDENTITY_PROTOCOL = "tuojing_profile_arm_ids_v3"
REAL_SKILL_REVISION = "2a722748fa2adedd096c8fd7f9461d2b2ba48af6"


def arm_identities(config: TaskConfig) -> dict[str, dict[str, str]]:
    """Fixed base identities, not a transform of live TCPs or task instructions.

    The real skill names arms from the robot's perspective. L1 uses those names
    directly; common L2/L3 profiles retain overview-based command names. Derive
    identity from the bound physical base, never from the command name alone.
    Do not swap task roles again when decoding actions.
    Other robot layouts do not inherit this lab-specific correspondence.
    """
    if config.raw.get("simulation", {}).get("environment") != "tuojing_lab_v4":
        return {}
    base_sides = {"/World/Robots/R5_Left": "left", "/World/Robots/R5_Right": "right"}
    sides = {mount.name: base_sides[mount.prim_path] for mount in config.robot.mounts}
    real_arm = {"left": "right", "right": "left"}
    real_camera = {"left": "gemini_right", "right": "gemini"}
    return {
        arm: {"command_arm": arm, "overview_base_side": sides[arm],
              "real_arm": real_arm[sides[arm]], "wrist_camera": arm + "_wrist",
              "real_wrist_camera_key": real_camera[sides[arm]], "role": role}
        for arm, role in config.arm_roles.items()
    }


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
        if self.max_drift > self.tolerance and assessment.status != "invalid":
            return Assessment("failed", "Fixed arm moved beyond the configured tolerance", metrics)
        return replace(assessment, metrics=metrics)
