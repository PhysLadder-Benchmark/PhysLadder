from ..config import ExecutionSettings, TaskConfig
from ..interfaces import Backend
from ..types import Action, ArmState, Feedback, InvalidSimulationState
from .audit import require_joint_sim_ready
from .control import bounded_measured_joints, joints_to_width, plan_targets, width_to_joints
from .settling import SettlingWindow


class RobotAdapter:
    def initialize(self, backend: Backend, config: TaskConfig) -> None:
        raise NotImplementedError(
            "Load robot asset, joints, limits, gripper and controllers"
        )

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
    Cartesian goals use the shared cuRobo planner followed by bounded drive tracking.
    Gripper completion describes execution, never evidence of task success.
    """

    def __init__(self) -> None:
        self.backend = self.config = None
        self.instances = {}
        self.indices = {}
        self.targets = {}
        self.initial_targets = {}
        self.drive_targets = {}
        self.kinematics = None
        self.cartesian_targets = {}
        self.rejection = None
        self.pending_grippers = {}
        self.motion_arms = set()
        self.motion_stable_s = 0.0
        self.execution = ExecutionSettings()
        self.planner = None
        self.paths = {}
        self.path_time_s = 0.0
        self.planning_diagnostics = {}
        self.last_plan = None
        self.grip_elapsed = {}
        self.grip_durations = {}
        self.grip_closing = {}
        self.grip_arms = set()
        self.settling = SettlingWindow()
        self.tracking_time_s = 0.0

    def initialize(self, backend: Backend, config: TaskConfig) -> None:
        from ..backends.isaacsim import IsaacSimBackend

        require_joint_sim_ready(config.robot)
        if not isinstance(backend, IsaacSimBackend):
            raise TypeError(
                "R5Adapter requires IsaacSimBackend; it never connects to real hardware"
            )
        self.config, self.backend = config, backend
        self.execution = ExecutionSettings.from_config(config)
        from .kinematics import R5Kinematics
        from ..planning.client import PlannerClient

        self.kinematics = R5Kinematics(config.robot)
        self.instances = backend.add_r5(config.robot)
        self.planner = PlannerClient(
            config.robot,
            backend.settings.asset_directory / "curobo",
            log_directory=getattr(backend, "run_directory", None),
        )
        backend.scene_report["ik_solver"] = "curobo"
        backend.scene_report["motion_planner"] = self.planner.metadata
        backend.before_physics_step = self.before_physics_step

    def reset(self, seed: int) -> None:
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        profile = self.config.robot
        self.settling = SettlingWindow()
        self.tracking_time_s = 0.0
        self.paths.clear()
        self.drive_targets.clear()
        self.grip_elapsed.clear()
        self.grip_durations.clear()
        self.grip_closing.clear()
        self.grip_arms.clear()
        self.path_time_s = 0.0
        self.planning_diagnostics = {}
        self.last_plan = None
        self.cartesian_targets.clear()
        self.rejection = None
        self.pending_grippers.clear()
        self.motion_arms.clear()
        self.motion_stable_s = 0.0
        ordered = profile.joint_names + profile.gripper.joint_names
        limits = {limit.joint: limit for limit in profile.operating_limits}
        width = sum(profile.gripper.width_limits_m) / 2
        self.backend.scene_report["gripper_execution"] = {
            "protocol": "rate_limited_drive_reference_v1",
            "joint_reference_speed_m_s": {n: limits[n].velocity for n in profile.gripper.joint_names},
            "timeout": "hold_last_applied_reference",
            "scope": "Limits commanded reference speed; contact impulses can cause measured velocity transients.",
        }
        for mount in profile.mounts:
            arm = self.instances[mount.name]
            actual = tuple(arm.dof_names)
            if len(actual) != len(ordered) or set(actual) != set(ordered):
                raise ValueError(
                    f"{mount.name}: USD joint names do not match profile: {actual}"
                )
            indices = np.array([actual.index(name) for name in ordered], dtype=np.int32)
            self.indices[mount.name] = indices
            target = tuple(mount.home_positions_rad) + width_to_joints(profile, width)
            self.targets[mount.name] = self.initial_targets[mount.name] = target
            self.drive_targets[mount.name] = target
            settings = self.backend.settings
            controller = arm.get_articulation_controller()
            stiffness, damping = np.empty(8), np.empty(8)
            stiffness[indices] = [settings.arm_stiffness] * 6 + [
                settings.finger_stiffness
            ] * 2
            damping[indices] = [settings.arm_damping] * 6 + [
                settings.finger_damping
            ] * 2
            controller.set_gains(kps=stiffness, kds=damping)
            controller.set_max_efforts(
                np.array([limits[name].effort for name in ordered]),
                joint_indices=indices,
            )
            arm.set_joint_positions(np.array(target), joint_indices=indices)
            arm.set_joint_velocities(np.zeros(8), joint_indices=indices)
            arm.apply_action(
                ArticulationAction(
                    joint_positions=np.array(target), joint_indices=indices
                )
            )
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
                plan_targets(
                    self.config.robot, self.config.command_arms, action, self.targets
                )
                # This new wait was accepted. Report current execution feedback,
                # not the rejection of a previous decision or its old plan.
                self.rejection = None
                self.last_plan = None
                return
            states = self.state()
            # Validate all non-Cartesian fields before calling the planner.
            from dataclasses import replace

            validated = replace(
                action,
                commands=tuple(
                    c for c in action.commands if c.kind != "cartesian_pose"
                ),
            )
            planned = plan_targets(
                self.config.robot, self.config.command_arms, validated, self.targets
            )
            motion = tuple(c for c in action.commands if c.kind != "gripper")
            paths = {}
            diagnostics = {}
            if motion:
                from ..planning.scene import snapshot
                from ..planning.trajectory import JointPath

                reply = self.planner.plan(action, states, snapshot(self.backend))
                for arm, data in reply["paths"].items():
                    limits = [
                        next(
                            x
                            for x in self.config.robot.operating_limits
                            if x.joint == name
                        )
                        for name in self.config.robot.joint_names
                    ]
                    paths[arm] = JointPath.from_timed(
                        data["positions"], data["times_s"], states[arm].joint_positions_rad, limits
                    )
                    # Preserve the exact time law already checked for BOTH arms.
                    planned[arm] = paths[arm].positions[-1] + planned[arm][6:]
                if set(paths) != {c.arm for c in motion}:
                    raise ValueError("Planner did not return every requested arm")
                if (
                    max(p.duration_s for p in paths.values())
                    > (
                        self.execution.max_action_steps
                        - self.execution.completion_samples
                    )
                    * self.config.control_dt_s
                    - 1.0
                ):
                    raise ValueError(
                        "Planned motion exceeds per-action budget; use a closer target"
                    )
                diagnostics = reply["diagnostics"]
            cartesian_targets = dict(self.cartesian_targets)
            for command in action.commands:
                if command.kind == "cartesian_pose":
                    cartesian_targets[command.arm] = self.kinematics.world_pose(
                        command.values, command.arm, command.frame
                    )
                elif command.kind == "joint_position":
                    cartesian_targets.pop(command.arm, None)
        except ValueError as exc:
            # Retain previous targets for BOTH arms, expose feedback, and propagate the error.
            metrics = {
                "reason_code": "invalid_target",
                "requested_commands": [
                    {"arm": c.arm, "kind": c.kind, "values": c.values, "frame": c.frame}
                    for c in getattr(action, "commands", ())
                ],
            }
            from ..planning.failures import PlanningFailure

            if isinstance(exc, PlanningFailure):
                metrics["reason_code"] = exc.diagnostic["code"]
                metrics["planning_failure"] = exc.diagnostic
            self.rejection = Feedback("rejected", str(exc), metrics)
            raise
        self.pending_grippers = {}
        self.paths = paths
        self.path_time_s = 0.0
        self.planning_diagnostics = diagnostics
        self.last_plan = {
            "paths": {
                name: {"positions": path.positions, "times_s": path.times_s}
                for name, path in paths.items()
            },
            "diagnostics": diagnostics,
        }
        self.motion_arms = {c.arm for c in action.commands if c.kind != "gripper"}
        self.grip_arms = {c.arm for c in action.commands if c.kind == "gripper"}
        initial = dict(planned)
        for name, path in paths.items():
            initial[name] = path.positions[0] + initial[name][6:]
        for c in action.commands:
            if c.kind == "gripper":
                if c.arm in self.motion_arms:
                    self.pending_grippers[c.arm] = planned[c.arm][6:]
                    initial[c.arm] = initial[c.arm][:6] + self.targets[c.arm][6:]
                else:
                    self._start_grip(c.arm, planned[c.arm][6:], states[c.arm])
        self.motion_stable_s = 0.0
        self._send_targets(initial)
        self.cartesian_targets = cartesian_targets
        self.rejection = None

    def _start_grip(self, arm, target, state):
        travel = max(abs(a - b) for a, b in zip(target, state.finger_positions_m))
        speed = min(
            x.velocity
            for x in self.config.robot.operating_limits
            if x.joint in self.config.robot.gripper.joint_names
        )
        self.grip_elapsed[arm] = 0.0
        self.grip_durations[arm] = max(0.5, travel / speed + 0.5)
        self.grip_closing[arm] = sum(target) < sum(state.finger_positions_m) - 0.001

    def close(self):
        if self.planner is not None:
            self.planner.close()
            self.planner = None

    def _send_targets(self, planned):
        # Keep requested goals separate from the rate-limited drive reference.
        self.targets = planned
        self._advance_drive_targets(0.0)

    def _advance_drive_targets(self, dt_s):
        if not self.instances:
            return
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        limits = {limit.joint: limit.velocity
                  for limit in self.config.robot.operating_limits}
        speeds = tuple(limits[name] for name in self.config.robot.gripper.joint_names)
        for name, goal in self.targets.items():
            previous = self.drive_targets[name]
            fingers = tuple(
                old + max(-speed * dt_s, min(speed * dt_s, desired - old))
                for old, desired, speed in zip(previous[6:], goal[6:], speeds)
            )
            reference = goal[:6] + fingers
            if reference != previous:
                self.instances[name].apply_action(ArticulationAction(
                    joint_positions=np.array(reference), joint_indices=self.indices[name]))
                self.drive_targets[name] = reference

    def before_physics_step(self, dt_s: float) -> None:
        self.tracking_time_s += dt_s
        for name, arm in self.instances.items():
            self.settling.add(self.tracking_time_s, name,
                             arm.get_joint_positions(joint_indices=self.indices[name]))
        for arm in self.grip_elapsed:
            self.grip_elapsed[arm] += dt_s
        if self.paths:
            measured = self.state()
            tracking_lag = max(
                abs(actual - target)
                for name in self.paths
                for actual, target in zip(
                    measured[name].joint_positions_rad, self.targets[name][:6]
                )
            )
            # A lagging drive must catch up before the reference advances.
            # Persistent obstruction remains bounded by the per-action timeout.
            if tracking_lag <= 0.10:
                self.path_time_s += dt_s
            targets = dict(self.targets)
            finished = []
            for arm, path in self.paths.items():
                targets[arm] = path.sample(self.path_time_s) + targets[arm][6:]
                if self.path_time_s >= path.duration_s:
                    finished.append(arm)
            self._send_targets(targets)
            for arm in finished:
                del self.paths[arm]
        if self.pending_grippers and not self.paths:
            feedback = self._tracking_feedback()
            motion_ready = all(
                feedback.metrics["arms"][name]["motion_reached"]
                for name in self.motion_arms
            )
            self.motion_stable_s = self.motion_stable_s + dt_s if motion_ready else 0.0
            if (
                self.motion_stable_s
                >= self.execution.completion_samples * self.config.control_dt_s
            ):
                targets = dict(self.targets)
                states = self.state()
                for name, fingers in self.pending_grippers.items():
                    targets[name] = targets[name][:6] + fingers
                    self._start_grip(name, fingers, states[name])
                self.pending_grippers.clear()
                self._send_targets(targets)

        self._advance_drive_targets(dt_s)

    def on_action_timeout(self) -> None:
        """Cancel queued motion/closing; preserve the current bounded grip drive."""
        self.pending_grippers.clear()
        self.paths.clear()
        # The timed-out Cartesian goal is cancelled, not an outstanding demand
        # that can poison completion feedback for a later gripper-only action.
        for name in self.motion_arms:
            self.cartesian_targets.pop(name, None)
        states = self.state()
        limits_by_name = {x.joint: x for x in self.config.robot.operating_limits}
        limits = [limits_by_name[n] for n in self.config.robot.joint_names]
        self._send_targets(
            {
                name: bounded_measured_joints(states[name].joint_positions_rad, limits)
                + self.drive_targets.get(name, target)[6:]
                for name, target in self.targets.items()
            }
        )

    def state(self) -> dict[str, ArmState]:
        import math

        states = {}
        for name, arm in self.instances.items():
            indices = self.indices[name]
            positions = tuple(
                float(v) for v in arm.get_joint_positions(joint_indices=indices)
            )
            velocities = tuple(
                float(v) for v in arm.get_joint_velocities(joint_indices=indices)
            )
            if not all(math.isfinite(value) for value in positions + velocities):
                raise InvalidSimulationState(f"{name}: nonfinite R5 joint state")
            tcp_pose = None
            if self.kinematics is not None:
                from .transforms import pose_transform, transform_pose

                tcp_pose = transform_pose(
                    pose_transform(self.backend.read_ee_pose(name))
                    @ self.kinematics.tcp
                )
            states[name] = ArmState(
                joint_positions_rad=positions[:6],
                joint_velocities_rad_s=velocities[:6],
                gripper_width_m=joints_to_width(self.config.robot, positions[6:]),
                finger_positions_m=positions[6:],
                finger_velocities_m_s=velocities[6:],
                tcp_pose_world_wxyz=tcp_pose,
            )
        return states

    def feedback(self) -> Feedback:
        if not self.targets:
            return Feedback("not_initialized", "R5 state is unavailable before reset")
        if self.rejection is not None:
            return self.rejection
        return self._tracking_feedback()

    def _tracking_feedback(self) -> Feedback:
        """Controller feedback independent of the last policy-request rejection."""
        errors = {}
        tcp_errors = {}
        details = {}
        settled = True
        for name, state in self.state().items():
            target = self.targets[name]
            arm_error = max(
                abs(q - desired)
                for q, desired in zip(state.joint_positions_rad, target[:6])
            )
            finger_error = max(
                abs(q - desired)
                for q, desired in zip(state.finger_positions_m, target[6:])
            )
            errors[name] = (arm_error, finger_error)
            if name in self.cartesian_targets:
                from .transforms import pose_errors

                if state.tcp_pose_world_wxyz is None:
                    return Feedback(
                        "error", f"{name}: measured TCP pose is unavailable"
                    )
                position_error, orientation_error = pose_errors(
                    state.tcp_pose_world_wxyz, self.cartesian_targets[name]
                )
                tcp_errors[name] = {
                    "position_error_m": position_error,
                    "orientation_error_rad": orientation_error,
                    "requested_pose_world_wxyz": self.cartesian_targets[name],
                    "measured_pose_world_wxyz": state.tcp_pose_world_wxyz,
                    "remaining_xyz_m": tuple(
                        t - q
                        for t, q in zip(
                            self.cartesian_targets[name][:3],
                            state.tcp_pose_world_wxyz[:3],
                        )
                    ),
                }
            window_arm, window_fingers = self.settling.settled(
                self.tracking_time_s, name, state.joint_positions_rad + state.finger_positions_m)
            arm_settled = all(abs(v) < 0.02 for v in state.joint_velocities_rad_s) or window_arm
            cartesian_reached = name not in tcp_errors or (
                position_error < 0.005 and orientation_error < 0.02
            )
            desired_width = joints_to_width(
                self.config.robot, self.pending_grippers.get(name, target[6:])
            )
            gripper_reached = (
                finger_error < 0.0015 and name not in self.pending_grippers
                and self.drive_targets.get(name, target)[6:] == target[6:]
            )
            fingers_still = all(abs(v) < 0.003 for v in state.finger_velocities_m_s) or window_fingers
            closing_obstructed = (
                self.grip_closing.get(name, False)
                and not gripper_reached
                and name not in self.pending_grippers
                and fingers_still
                and self.grip_elapsed.get(name, 0.0)
                >= self.grip_durations.get(name, float("inf"))
                and state.gripper_width_m is not None
                and state.gripper_width_m > desired_width + 0.0015
            )
            motion_reached = (
                arm_error < 0.015
                and arm_settled
                and cartesian_reached
                and name not in self.paths
            )
            gripper_complete = fingers_still and (gripper_reached or closing_obstructed)
            details[name] = {
                "position_window_settled": {"arm": window_arm, "fingers": window_fingers, "duration_s": .25},
                "motion_reached": motion_reached,
                "joint_error_max_rad": arm_error,
                "finger_error_max_m": finger_error,
                "requested_gripper_width_m": desired_width,
                "measured_gripper_width_m": state.gripper_width_m,
                "gripper_width_error_m": (
                    None
                    if state.gripper_width_m is None
                    else desired_width - state.gripper_width_m
                ),
                "gripper_reached": gripper_reached,
                "gripper_execution_complete": gripper_complete,
                "gripper_state": (
                    "pending_motion"
                    if name in self.pending_grippers
                    else "closing_obstructed_or_stalled"
                    if closing_obstructed
                    else "target_reached"
                    if gripper_reached
                    else "moving_or_not_reached"
                ),
                "grasp_confirmed": None,
                "empty_grasp_suspected": bool(
                    self.grip_closing.get(name, False)
                    and gripper_complete and state.gripper_width_m is not None
                    and state.gripper_width_m <= self.config.robot.gripper.width_limits_m[0] + .0015
                ),
                "grasp_evidence": (
                    "Not determined by controller; verify contact and object response using images "
                    "and a small task-appropriate movement along its allowed axis; do not lift a constrained part"
                    if self.config.grasp_check_mode == "contact_inspection"
                    else "Not determined by controller; verify object following a short lift in images"
                ),
                "gripper_pending": name in self.pending_grippers,
            }
            settled = settled and motion_reached and gripper_complete
        complete = settled and not self.pending_grippers and not self.paths
        return Feedback(
            "completed" if complete else "executing",
            f"Joint tracking errors (rad, m): {errors}",
            {
                "tcp": tcp_errors,
                "arms": details,
                "planning": self.planning_diagnostics,
                "phase": "trajectory_tracking"
                if self.paths
                else "motion_before_gripper"
                if self.pending_grippers
                else "target_tracking",
            },
        )
