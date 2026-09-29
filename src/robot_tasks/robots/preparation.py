"""Fixed robot-only episode preparation; never reads task object state."""

import math

from .transforms import pose_transform
from ..types import Action, ArmCommand


def preparation_action(config):
    """Prepare manipulation arms only; observation arms keep their reset pose.

    The shared task launcher always supplies this action before model observation.
    It uses the existing Lula executor, with no task-dependent motion mode.
    """
    commands = []
    for name in config.arms:
        mount = next(m for m in config.robot.mounts if m.name == name)
        base = pose_transform(mount.base_pose_wxyz)
        # Fixed offset from the base, independent of all task assets/targets.
        # With the shared left mount this is world (0.36, 0.42, 0.25) metres.
        xyz = base[:3, :3] @ (0.36, 0.0, 0.21) + base[:3, 3]
        down = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
        commands.append(ArmCommand(name, "cartesian_pose", (*map(float, xyz), *down)))
    # No gripper command: preparation preserves the existing opening.
    return Action(tuple(commands), config.control_dt_s)
