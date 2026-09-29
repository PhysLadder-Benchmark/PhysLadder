from ..config import ExecutionSettings, TaskConfig
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
    Cartesian goals use bounded IK followed by joint physics drives.
    This adapter does not provide collision-free path planning or force control.
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
        self.pending_grippers = {}
        self.motion_arms = set()
        self.motion_stable_s = 0.0
        self.execution = ExecutionSettings()
        self.motion_paths = {}
        self.opening_arm_targets = {}
        self.bench_guard = None
        self.gripper_progress = {}
        self.gripper_intents = {}
        self.blocked_feedback = None
        self.arm_stability = {}

    def initialize(self, backend: Backend, config: TaskConfig) -> None:
        from ..backends.isaacsim import IsaacSimBackend
        require_joint_sim_ready(config.robot)
        if not isinstance(backend, IsaacSimBackend):
            raise TypeError("R5Adapter requires IsaacSimBackend; it never connects to real hardware")
        self.config, self.backend = config, backend
        self.execution = ExecutionSettings.from_config(config)
        if config.robot.tcp_pose_wxyz is not None:
            from .kinematics import R5Kinematics
            self.kinematics = R5Kinematics(config.robot)
        self.instances = backend.add_r5(config.robot)
        backend.scene_report["ik_solver"] = "numerical"
        if self.execution.motion_mode == "cartesian_linear_v1":
            from .motion import BenchGuard, VERSION, LINEAR_SPEED_M_S, ANGULAR_SPEED_RAD_S
            self.bench_guard = BenchGuard(config.robot)
            backend.scene_report["motion_executor"] = {
                "version": VERSION, "linear_speed_m_s": LINEAR_SPEED_M_S,
                "angular_speed_rad_s": ANGULAR_SPEED_RAD_S,
                "collision_check": "conservative robot envelopes against known bench only",
                "gripper_timing": self.execution.gripper_timing,
                "gripper_completion": self.execution.gripper_completion,
            }
        backend.before_physics_step = self.before_physics_step

    def reset(self, seed: int) -> None:
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        profile = self.config.robot
        self.cartesian_targets.clear()
        self.rejection = None
        self.pending_grippers.clear()
        self.motion_arms.clear()
        self.motion_stable_s = 0.0
        self.motion_paths.clear()
        self.opening_arm_targets.clear()
        self.gripper_progress.clear()
        self.arm_stability.clear()
        self.gripper_intents.clear()
        self.blocked_feedback = None
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
            self.gripper_intents[mount.name] = width
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
            # Validate executable fields before solving and changing drive targets.
            from ..runner import validate_action
            validate_action(action, self.config, require_control_duration=False)
            if not action.commands:
                plan_targets(self.config.robot, self.config.arms, action, self.targets)
                return
            needs_state = (any(c.kind == "cartesian_pose" for c in action.commands)
                           or self.execution.gripper_timing == "open_before_motion")
            states = self.state() if needs_state else {}
            measured = ({name: state.joint_positions_rad for name, state in states.items()}
                        if needs_state else None)
            planned = plan_targets(self.config.robot, self.config.arms, action, self.targets,
                                   kinematics=self.kinematics, measured_positions=measured)
            paths = {}
            if self.execution.motion_mode == "cartesian_linear_v1":
                from .motion import plan_cartesian
                if self.bench_guard is None:
                    raise RuntimeError("Cartesian path guard was not initialized")
                for command in action.commands:
                    if command.kind != "cartesian_pose":
                        continue
                    arm = command.arm
                    fingers = tuple(max(a, b) for a, b in zip(states[arm].finger_positions_m, planned[arm][6:]))
                    goal = self.kinematics.world_pose(command.values, arm, command.frame)
                    paths[arm] = plan_cartesian(
                        self.kinematics, self.bench_guard, arm, measured[arm], goal, fingers,
                        budget_s=self.execution.max_action_steps * self.config.control_dt_s - 1.0,
                    )
                    planned[arm] = tuple(paths[arm].points[-1]) + planned[arm][6:]
            cartesian_targets = dict(self.cartesian_targets)
            for command in action.commands:
                if command.kind == "cartesian_pose":
                    cartesian_targets[command.arm] = self.kinematics.world_pose(
                        command.values, command.arm, command.frame)
                elif command.kind == "joint_position":
                    cartesian_targets.pop(command.arm, None)
        except ValueError as exc:
            # Retain previous targets for BOTH arms, expose feedback, and propagate the error.
            from .kinematics import IKError
            metrics = {"reason_code": "invalid_target", "requested_commands": [
                {"arm": c.arm, "kind": c.kind, "values": c.values, "frame": c.frame} for c in getattr(action, "commands", ())]}
            if isinstance(exc, IKError):
                from dataclasses import asdict
                metrics.update(reason_code=exc.result.reason, ik=asdict(exc.result))
            self.rejection = Feedback("rejected", str(exc), metrics)
            raise
        self.pending_grippers = {}
        self.motion_paths = paths
        self.opening_arm_targets = {}
        self.motion_arms = {c.arm for c in action.commands if c.kind != "gripper"}
        if self.execution.gripper_timing in {"after_motion", "open_before_motion"}:
            for c in action.commands:
                if c.kind == "gripper" and c.arm in self.motion_arms:
                    if (self.execution.gripper_completion == "stall_aware_v1"
                            and abs(c.values[0] - self.gripper_intents.get(
                                c.arm, joints_to_width(self.config.robot, self.targets[c.arm][6:]))) < 1e-9):
                        # Preserve the existing drive while carrying an object;
                        # never wait for a second close to an unchanged target.
                        planned[c.arm] = planned[c.arm][:6] + self.targets[c.arm][6:]
                        continue
                    if (self.execution.gripper_timing == "open_before_motion"
                            and c.values[0] > states[c.arm].gripper_width_m + 0.0015
                            and (self.execution.gripper_completion == "position"
                                 or c.values[0] > self.gripper_intents.get(
                                     c.arm, joints_to_width(self.config.robot, self.targets[c.arm][6:])) + 0.0015)):
                        self.opening_arm_targets[c.arm] = planned[c.arm][:6]
                        planned[c.arm] = states[c.arm].joint_positions_rad + planned[c.arm][6:]
                    else:
                        self.pending_grippers[c.arm] = planned[c.arm][6:]
                        planned[c.arm] = planned[c.arm][:6] + self.targets[c.arm][6:]
        # Begin a path at measured joints, never jump directly to the final IK solution.
        for arm, path in self.motion_paths.items():
            planned[arm] = tuple(path.points[0]) + planned[arm][6:]
        self.motion_stable_s = 0.0
        self._send_targets(planned)
        self.cartesian_targets = cartesian_targets
        self.rejection = None
        self.blocked_feedback = None
        for command in action.commands:
            if command.kind == "gripper":
                self.gripper_intents[command.arm] = command.values[0]

    def _send_targets(self, planned):
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

    def before_physics_step(self, dt_s: float) -> None:
        if self.execution.gripper_completion == "stall_aware_v1" and self.targets:
            from .gripper import GripperProgress, PositionStability
            for arm, state in self.state().items():
                self.arm_stability.setdefault(arm, PositionStability(.003)).update(state.joint_positions_rad, dt_s)
                progress = self.gripper_progress.setdefault(arm, GripperProgress())
                progress.update(state.finger_positions_m, state.finger_velocities_m_s,
                                self.targets[arm][6:], dt_s)
                if arm in self.opening_arm_targets and progress.status in {"opening_stalled", "stalled"}:
                    # A failed opening must not later release a queued arm motion.
                    feedback = self._tracking_feedback()
                    cancellation = self.on_action_timeout()
                    self.blocked_feedback = Feedback(
                        "blocked", "Opening stalled; queued motion cancelled. Inspect RGB/state. "
                        "A new arm-only retreat preserves the current gripper target.",
                        {**feedback.metrics, **cancellation, "reason_code": "opening_stalled"})
                    return
        if self.opening_arm_targets or self.motion_paths:
            states = self.state()
            targets = dict(self.targets)
            for arm in list(self.opening_arm_targets):
                state = states[arm]
                fingers_ready = (max(abs(q - t) for q, t in zip(state.finger_positions_m, targets[arm][6:])) < 0.0015
                                 and all(abs(v) < 0.003 for v in state.finger_velocities_m_s))
                if fingers_ready:
                    endpoint = self.opening_arm_targets.pop(arm)
                    if arm not in self.motion_paths:
                        targets[arm] = endpoint + targets[arm][6:]
            for arm, path in list(self.motion_paths.items()):
                if arm in self.opening_arm_targets:
                    continue
                q = path.advance(dt_s, states[arm].joint_positions_rad)
                targets[arm] = q + targets[arm][6:]
                if path.finished:
                    del self.motion_paths[arm]
            self._send_targets(targets)
        if self.pending_grippers:
            feedback = self._tracking_feedback()
            motion_ready = all(feedback.metrics["arms"][name]["motion_reached"] for name in self.motion_arms)
            self.motion_stable_s = self.motion_stable_s + dt_s if motion_ready else 0.0
            if self.motion_stable_s >= self.execution.completion_samples * self.config.control_dt_s:
                targets = dict(self.targets)
                for name, fingers in self.pending_grippers.items():
                    targets[name] = targets[name][:6] + fingers
                self.pending_grippers.clear()
                self._send_targets(targets)

    def on_action_timeout(self) -> dict[str, bool]:
        """Never trigger a deferred grip after its action budget has expired."""
        self.pending_grippers.clear()
        if self.execution.motion_mode == "cartesian_linear_v1" or self.opening_arm_targets:
            # Cancel queued motion too: a later wait must not resume a failed path.
            self.motion_paths.clear()
            self.opening_arm_targets.clear()
            states = self.state()
            targets = dict(self.targets)
            for arm in self.motion_arms:
                state = states[arm]
                targets[arm] = state.joint_positions_rad + targets[arm][6:]
                self.cartesian_targets.pop(arm, None)
            self._send_targets(targets)
            self.motion_arms.clear()
            return {"pending_motion_cancelled": True, "holding_measured_joints": True}
        return {"pending_gripper_cancelled": True}

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
        if self.blocked_feedback is not None:
            return self.blocked_feedback
        return self._tracking_feedback()

    def _tracking_feedback(self) -> Feedback:
        """Controller feedback independent of the last policy-request rejection."""
        errors = {}
        tcp_errors = {}
        details = {}
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
                tcp_errors[name] = {"position_error_m": position_error, "orientation_error_rad": orientation_error,
                                    "requested_pose_world_wxyz": self.cartesian_targets[name],
                                    "measured_pose_world_wxyz": state.tcp_pose_world_wxyz,
                                    "remaining_xyz_m": tuple(t - q for t, q in zip(
                                        self.cartesian_targets[name][:3], state.tcp_pose_world_wxyz[:3]))}
            arm_settled = all(abs(v) < 0.02 for v in state.joint_velocities_rad_s)
            cartesian_reached = name not in tcp_errors or (position_error < .005 and orientation_error < .02)
            desired_width = joints_to_width(self.config.robot, self.pending_grippers.get(name, target[6:]))
            queued = name in self.motion_paths or name in self.opening_arm_targets
            progress = self.gripper_progress.get(name)
            grip_status = progress.status if progress else "moving"
            closing_stalled = (self.execution.gripper_completion == "stall_aware_v1"
                               and grip_status == "closing_stalled"
                               and progress.target == tuple(target[6:]))
            # Sustained joint-position stability is necessary to tolerate
            # solver velocity chatter during a blocked closing command.
            contact_settled = closing_stalled and self.arm_stability.get(name) is not None and self.arm_stability[name].quiet_s >= .5
            arm_settled = arm_settled or contact_settled
            details[name] = {"motion_reached": arm_error < .015 and arm_settled and cartesian_reached and not queued,
                             "joint_error_max_rad": arm_error, "finger_error_max_m": finger_error,
                             "requested_gripper_width_m": desired_width,
                             "measured_gripper_width_m": state.gripper_width_m,
                             "gripper_width_error_m": (None if state.gripper_width_m is None else desired_width - state.gripper_width_m),
                             "gripper_reached": finger_error < .0015 and name not in self.pending_grippers,
                             "gripper_state": "pending" if name in self.pending_grippers else grip_status,
                             "gripper_settled": (finger_error < .0015 or closing_stalled) and name not in self.pending_grippers,
                             "grasp_confirmed": None,
                             "gripper_pending": name in self.pending_grippers,
                             "path_pending": name in self.motion_paths,
                             "opening_before_motion": name in self.opening_arm_targets}
            settled = settled and arm_settled
            settled = settled and (all(abs(v) < 0.003 for v in state.finger_velocities_m_s) or contact_settled)
        complete = settled and all(arm < 0.015 and details[name]["gripper_settled"]
                                   for name, (arm, finger) in errors.items())
        complete = complete and all(error["position_error_m"] < 0.005 and error["orientation_error_rad"] < 0.02
                                    for error in tcp_errors.values())
        complete = complete and not (self.pending_grippers or self.motion_paths or self.opening_arm_targets)
        message = f"Joint tracking errors (rad, m): {errors}"
        if any(d["gripper_state"] == "closing_stalled" for d in details.values()):
            message += "; closing settled against resistance; object grasp is unverified, inspect RGB before transport."
        return Feedback("completed" if complete else "executing", message,
                        {"tcp": tcp_errors, "arms": details,
                         "phase": ("opening_before_motion" if self.opening_arm_targets else
                                   "cartesian_path" if self.motion_paths else
                                   "motion_before_gripper" if self.pending_grippers else "target_tracking")})
