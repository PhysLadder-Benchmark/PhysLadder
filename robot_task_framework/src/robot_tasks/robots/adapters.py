from ..config import TaskConfig
from ..interfaces import Backend
from ..types import Action, ArmState, Feedback
from .audit import require_joint_sim_ready
from .control import joints_to_width, plan_targets, width_to_joints


class RobotAdapter:
    def initialize(self, backend: Backend, config: TaskConfig) -> None:
        raise NotImplementedError("Load robot asset, joints, limits, gripper and controllers")

    def reset(self, seed: int) -> None:
        # Initial-state assignment is allowed during reset only.
        raise NotImplementedError("Robot initial configuration")

    def apply(self, action: Action) -> None:
        # TODO: validate limits, resolve frames; use IK for Cartesian targets.
        # Apply drive targets/forces, NEVER teleport objects to manufacture success.
        # For two arms, apply both targets before stepping the shared world.
        raise NotImplementedError("Action -> joint/Cartesian/gripper controller")

    def state(self) -> dict[str, ArmState]:
        raise NotImplementedError("Read robot state")

    def feedback(self) -> Feedback:
        raise NotImplementedError("Execution completion/error feedback")


class R5Adapter(RobotAdapter):
    """Dual-arm R5 SIMULATION adapter; never connects to the real robot SDK.

    The manufacturer description and names are configured in config.robot.
    Cartesian goals use bounded IK, then the same joint physics drives.
    No straight-line Cartesian path, collision planning, or force control is implied.
    """

    def __init__(self) -> None:
        self.backend = self.config = None
        self.instances = {}
        self.indices = {}
        self.targets = {}
        self.initial_targets = {}
        self.kinematics = None
        self.cartesian_targets = {}
        self.rejection = None

    def initialize(self, backend: Backend, config: TaskConfig) -> None:
        from ..backends.isaacsim import IsaacSimBackend
        require_joint_sim_ready(config.robot)
        if not isinstance(backend, IsaacSimBackend):
            raise TypeError("R5Adapter requires IsaacSimBackend; it never connects to real hardware")
        self.config, self.backend = config, backend
        if config.robot.tcp_pose_wxyz is not None:
            from .kinematics import R5Kinematics
            self.kinematics = R5Kinematics(config.robot)
        self.instances = backend.add_r5(config.robot)

    def reset(self, seed: int) -> None:
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        profile = self.config.robot
        self.cartesian_targets.clear()
        self.rejection = None
        ordered = profile.joint_names + profile.gripper.joint_names
        limits = {limit.joint: limit for limit in profile.operating_limits}
        width = sum(profile.gripper.width_limits_m) / 2
        for mount in profile.mounts:
            arm = self.instances[mount.name]
            actual = tuple(arm.dof_names)
            if len(actual) != len(ordered) or set(actual) != set(ordered):
                raise ValueError(f"{mount.name}: USD joint names do not match profile: {actual}")
            indices = np.array([actual.index(name) for name in ordered], dtype=np.int32)
            self.indices[mount.name] = indices
            target = tuple(mount.home_positions_rad) + width_to_joints(profile, width)
            self.targets[mount.name] = self.initial_targets[mount.name] = target
            settings = self.backend.settings
            controller = arm.get_articulation_controller()
            stiffness, damping = np.empty(8), np.empty(8)
            stiffness[indices] = [settings.arm_stiffness] * 6 + [settings.finger_stiffness] * 2
            damping[indices] = [settings.arm_damping] * 6 + [settings.finger_damping] * 2
            controller.set_gains(kps=stiffness, kds=damping)
            controller.set_max_efforts(np.array([limits[name].effort for name in ordered]), joint_indices=indices)
            arm.set_joint_positions(np.array(target), joint_indices=indices)
            arm.set_joint_velocities(np.zeros(8), joint_indices=indices)
            arm.apply_action(ArticulationAction(joint_positions=np.array(target), joint_indices=indices))
            self.backend.scene_report["arms"][mount.name]["dof_names"] = actual

    def apply(self, action: Action) -> None:
        if not self.targets:
            raise RuntimeError("Reset the R5 robot before applying commands")
        try:
            if not isinstance(action, Action):
                raise ValueError("Expected an Action")
            measured = ({name: state.joint_positions_rad for name, state in self.state().items()}
                        if any(c.kind == "cartesian_pose" for c in action.commands) else None)
            planned = plan_targets(self.config.robot, self.config.arms, action, self.targets,
                                   kinematics=self.kinematics, measured_positions=measured)
            cartesian_targets = dict(self.cartesian_targets)
            for command in action.commands:
                if command.kind == "cartesian_pose":
                    cartesian_targets[command.arm] = self.kinematics.world_pose(
                        command.values, command.arm, command.frame)
                elif command.kind == "joint_position":
                    cartesian_targets.pop(command.arm, None)
        except ValueError as exc:
            # Retain previous targets for BOTH arms, expose feedback, and propagate the error.
            self.rejection = Feedback("rejected", str(exc))
            raise
        changed = {name: target for name, target in planned.items() if target != self.targets[name]}
        if changed:
            import numpy as np
            from isaacsim.core.utils.types import ArticulationAction

            # PhysX retains drive targets. Re-sending unchanged targets needlessly
            # wakes sleeping articulations, including the inactive/holding arm.
            for name, target in changed.items():
                self.instances[name].apply_action(ArticulationAction(
                    joint_positions=np.array(target), joint_indices=self.indices[name],
                ))
        self.targets = planned
        self.cartesian_targets = cartesian_targets
        if action.commands:
            self.rejection = None

    def state(self) -> dict[str, ArmState]:
        import math

        states = {}
        for name, arm in self.instances.items():
            indices = self.indices[name]
            positions = tuple(float(v) for v in arm.get_joint_positions(joint_indices=indices))
            velocities = tuple(float(v) for v in arm.get_joint_velocities(joint_indices=indices))
            if not all(math.isfinite(value) for value in positions + velocities):
                raise RuntimeError(f"{name}: nonfinite R5 joint state")
            tcp_pose = None
            if self.kinematics is not None:
                from .transforms import pose_transform, transform_pose
                tcp_pose = transform_pose(pose_transform(self.backend.read_ee_pose(name)) @ self.kinematics.tcp)
            states[name] = ArmState(
                joint_positions_rad=positions[:6], joint_velocities_rad_s=velocities[:6],
                gripper_width_m=joints_to_width(self.config.robot, positions[6:]),
                finger_positions_m=positions[6:], finger_velocities_m_s=velocities[6:],
                tcp_pose_world_wxyz=tcp_pose,
            )
        return states

    def feedback(self) -> Feedback:
        if not self.targets:
            return Feedback("not_initialized", "R5 state is unavailable before reset")
        if self.rejection is not None:
            return self.rejection
        errors = {}
        tcp_errors = {}
        settled = True
        for name, state in self.state().items():
            target = self.targets[name]
            arm_error = max(abs(q - desired) for q, desired in zip(state.joint_positions_rad, target[:6]))
            finger_error = max(abs(q - desired) for q, desired in zip(state.finger_positions_m, target[6:]))
            errors[name] = (arm_error, finger_error)
            if name in self.cartesian_targets:
                from .transforms import pose_errors
                if state.tcp_pose_world_wxyz is None:
                    return Feedback("error", f"{name}: measured TCP pose is unavailable")
                position_error, orientation_error = pose_errors(state.tcp_pose_world_wxyz, self.cartesian_targets[name])
                tcp_errors[name] = {"position_error_m": position_error, "orientation_error_rad": orientation_error}
            settled = settled and all(abs(v) < 0.02 for v in state.joint_velocities_rad_s)
            settled = settled and all(abs(v) < 0.003 for v in state.finger_velocities_m_s)
        complete = settled and all(arm < 0.015 and finger < 0.0015 for arm, finger in errors.values())
        complete = complete and all(error["position_error_m"] < 0.005 and error["orientation_error_rad"] < 0.02
                                    for error in tcp_errors.values())
        return Feedback("completed" if complete else "executing", f"Joint tracking errors (rad, m): {errors}",
                        {"tcp": tcp_errors})
