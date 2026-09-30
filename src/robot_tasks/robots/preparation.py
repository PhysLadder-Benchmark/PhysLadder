"""Fixed robot-only episode preparation; never reads task object state."""

import math

import numpy as np

from .transforms import pose_transform, transform_pose
from ..cameras.profile import load_camera_rig
from ..types import Action, ArmCommand


def _observation_pose(base, manipulation_bases, focus, optical_mount, tcp_mount):
    """Aim the mounted camera, not the gripper axis, at the fixed work area.

    Stay on the observation arm's side of the midpoint, at most 0.32 m inward.
    For the standard R5 layout the optical centre is (0.28, -0.10, 0.50) m.
    These are engineering startup offsets, not a collision-free path planner.
    """
    inward = np.mean([b[:3, 3] for b in manipulation_bases], axis=0) - base[:3, 3]
    inward[2] = 0.0
    separation = np.linalg.norm(inward)
    if separation < 1e-6:
        raise ValueError("Observation preparation requires separated arm bases")
    eye = (base[:3, 3] + base[:3, 0] * 0.28
           + inward / separation * min(separation / 2, 0.32) + (0.0, 0.0, 0.46))
    forward = focus - eye
    distance = np.linalg.norm(forward)
    if distance < 1e-6:
        raise ValueError("Observation camera cannot coincide with its look-at point")
    forward /= distance
    right = np.cross(forward, (0.0, 0.0, 1.0))
    if np.linalg.norm(right) < 1e-6:
        raise ValueError("Observation preparation needs a nonvertical viewing direction")
    right /= np.linalg.norm(right)
    camera = np.eye(4)
    # USD optical axes: +X right, +Y up, -Z forward.
    camera[:3, :3] = np.column_stack((right, np.cross(right, forward), -forward))
    camera[:3, 3] = eye
    # Both the optical mount and TCP are defined relative to the wrist link.
    return transform_pose(camera @ np.linalg.inv(optical_mount) @ tcp_mount)


def preparation_action(config):
    """Point manipulation arms down and observation cameras toward their work area.

    The shared task launcher always supplies this action before model observation.
    It uses the shared trajectory executor, with no task-dependent motion mode.
    """
    commands = []
    bases = {m.name: pose_transform(m.base_pose_wxyz) for m in config.robot.mounts}
    for name in config.arms:
        base = bases[name]
        # Fixed offset from the base, independent of all task assets/targets.
        # With the shared left mount this is world (0.36, 0.42, 0.25) metres.
        xyz = base[:3, :3] @ (0.36, 0.0, 0.21) + base[:3, 3]
        down = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
        commands.append(ArmCommand(name, "cartesian_pose", (*map(float, xyz), *down)))
    if config.observation_arms:
        rig = load_camera_rig(config.source.parent / config.raw["simulation"]["camera_profile"])
        cameras = rig.select(config.cameras)
        # Aim between the prepared gripper and the bench, without task object truth.
        focus = np.mean([c.values[:3] for c in commands], axis=0) - (0.0, 0.0, 0.12)
        for name in config.observation_arms:
            mounted = [c for c in cameras if c.parent == name]
            if len(mounted) != 1:
                raise ValueError(f"Observation arm {name} needs exactly one enabled wrist camera")
            target = _observation_pose(bases[name], [bases[a] for a in config.arms], focus,
                                       pose_transform(mounted[0].optical_pose_wxyz),
                                       pose_transform(config.robot.tcp_pose_wxyz))
            commands.append(ArmCommand(name, "cartesian_pose", target))
    # No gripper command: preparation preserves the existing opening.
    return Action(tuple(commands), config.control_dt_s)
