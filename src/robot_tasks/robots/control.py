"""Simulator-independent R5 action validation and gripper conversion."""

import math
from typing import TYPE_CHECKING

from ..types import Action
from .profile import RobotProfile

if TYPE_CHECKING:
    from .kinematics import R5Kinematics


def width_to_joints(profile: RobotProfile, width: float) -> tuple[float, float]:
    gripper = profile.gripper
    if (gripper.width_limits_m is None or gripper.joint_scale is None
            or gripper.joint_offset_m is None):
        raise ValueError("Gripper mapping has not been configured")
    if isinstance(width, bool) or not math.isfinite(width) or not gripper.width_limits_m[0] <= width <= gripper.width_limits_m[1]:
        raise ValueError("Gripper width is outside configured limits")
    return tuple(scale * width + offset for scale, offset in zip(gripper.joint_scale, gripper.joint_offset_m))


def joints_to_width(profile: RobotProfile, positions: tuple[float, ...]) -> float:
    gripper = profile.gripper
    if len(positions) != 2 or gripper.joint_scale is None or gripper.joint_offset_m is None:
        raise ValueError("Two finger positions and a calibrated mapping are required")
    # Average the two independent estimates; retain raw finger positions in state.
    return sum((q - offset) / scale for q, offset, scale in zip(
        positions, gripper.joint_offset_m, gripper.joint_scale)) / 2


def plan_targets(profile: RobotProfile, active_arms: tuple[str, ...], action: Action,
                 previous: dict[str, tuple[float, ...]], *,
                 kinematics: "R5Kinematics | None" = None,
                 measured_positions: dict[str, tuple[float, ...]] | None = None) -> dict[str, tuple[float, ...]]:
    """Validate the entire bimanual action before changing either arm's targets."""
    if (not isinstance(action, Action) or isinstance(action.duration_s, bool)
            or not math.isfinite(action.duration_s) or action.duration_s <= 0):
        raise ValueError("Expected an Action with positive finite duration")
    planned = dict(previous)
    channels = set()
    limits = {limit.joint: limit for limit in profile.operating_limits}
    joint_names = profile.joint_names + profile.gripper.joint_names
    for command in action.commands:
        if command.arm not in active_arms:
            raise ValueError(f"Arm is not active for this task: {command.arm}")
        if command.kind not in {"joint_position", "cartesian_pose", "gripper"}:
            raise ValueError(f"Unsupported R5 action kind: {command.kind}")
        channel = (command.arm, "gripper" if command.kind == "gripper" else "arm_motion")
        if channel in channels:
            raise ValueError(f"Duplicate action channel: {channel}")
        channels.add(channel)
        values = command.values
        count = {"joint_position": 6, "cartesian_pose": 7, "gripper": 1}[command.kind]
        if len(values) != count or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                      or not math.isfinite(v) for v in values):
            raise ValueError(f"{command.kind} requires {count} finite numeric values")
        current = planned[command.arm]
        if command.kind == "cartesian_pose":
            if kinematics is None or measured_positions is None:
                raise ValueError("Cartesian control requires configured kinematics and measured joint positions")
            from .kinematics import IKError
            solution = kinematics.solve(values, command.arm, measured_positions[command.arm], command.frame)
            if not solution.converged:
                raise IKError(solution)
            target = solution.positions_rad + current[6:]
        elif command.kind == "joint_position":
            target = tuple(values) + current[6:]
        else:
            target = current[:6] + width_to_joints(profile, values[0])
        for name, value in zip(joint_names, target):
            bound = limits[name]
            if not bound.lower <= value <= bound.upper:
                raise ValueError(f"{command.arm}.{name}: target outside configured limits")
        planned[command.arm] = target
    return planned
