"""Common episode lifecycle. It does not know any task-specific success rule."""

import math
import time
from dataclasses import asdict
from pathlib import Path

from .config import TaskConfig
from .interfaces import Backend, Policy, Robot, Task
from .recording import EpisodeRecorder
from .types import Action, Assessment, Feedback, Observation, PolicyInput


def validate_action(action: Action, config: TaskConfig) -> None:
    if not isinstance(action, Action):
        raise ValueError("Policy must return Action")
    if not math.isfinite(action.duration_s) or action.duration_s <= 0:
        raise ValueError("Action duration must be positive and finite")
    if not math.isclose(action.duration_s, config.control_dt_s, abs_tol=1e-9):
        raise ValueError("This scaffold uses a fixed control_dt_s per action")
    seen = set()
    for command in action.commands:
        if command.arm not in config.arms:
            raise ValueError(f"Unknown arm: {command.arm}")
        if command.kind not in {"joint_position", "cartesian_pose", "gripper"}:
            raise ValueError(f"Unsupported action kind: {command.kind}")
        channel = "gripper" if command.kind == "gripper" else "arm_motion"
        key = (command.arm, channel)
        if key in seen:
            raise ValueError("Conflicting targets for the same arm/channel")
        seen.add(key)
        if not command.values or not all(math.isfinite(v) for v in command.values):
            raise ValueError("Action values must be nonempty and finite")
        if command.kind == "cartesian_pose":
            if len(command.values) != 7 or command.frame not in {"world", "robot_base"}:
                raise ValueError("Pose needs xyz + quaternion(wxyz), and an explicit supported frame")
            norm = sum(v * v for v in command.values[3:])
            if not math.isclose(norm, 1.0, abs_tol=1e-3):
                raise ValueError("Pose quaternion must be normalized")
        if command.kind == "gripper" and (len(command.values) != 1 or command.values[0] < 0):
            raise ValueError("Gripper command is one nonnegative total opening width")
    # Joint count/limits, reachable workspace and force limits depend on the
    # actual robot. They MUST also be validated by Robot.apply before execution.


def run_episode(config: TaskConfig, backend: Backend, robot: Robot, task: Task,
                policy: Policy, output_root: Path, *, dry_run: bool = False,
                step_limit: int | None = None) -> tuple[Path, dict]:
    limit = config.max_steps if step_limit is None else step_limit
    if type(limit) is not int or not 0 < limit <= config.max_steps:
        raise ValueError("step_limit must be in [1, max_steps]")
    mode = "dry_run" if dry_run else "simulation"
    recorder = EpisodeRecorder(output_root, config, mode, limit)
    start = time.monotonic()
    assessment = Assessment("not_evaluated", "Episode has not been evaluated")
    completed_steps = 0
    status, reason, error = "error", "startup", None

    def observe() -> Observation:
        feedback = robot.feedback() if config.include_feedback else Feedback("not_provided")
        return Observation(backend.time_s(), backend.capture_rgb(),
                           robot.state() if config.include_robot_state else {}, feedback)

    try:
        policy_metadata = getattr(policy, "run_metadata", None)
        if policy_metadata is not None:
            recorder.event({"event": "policy_configuration", "configuration": policy_metadata})
        backend.open(config)
        task.build_scene(backend, config)
        robot.initialize(backend, config)
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        policy.reset(config.seed)
        scene_report = getattr(backend, "scene_report", None)
        if scene_report:
            recorder.event({"event": "simulation_configuration", "configuration": scene_report})
        observation = observe()
        recorder.event({"event": "reset", "observation": recorder.observation(observation, 0)})
        status, reason = "finished", "max_steps"
        for step in range(1, limit + 1):
            if time.monotonic() - start >= config.max_wall_time_s:
                reason = "wall_time_limit"
                break
            t0 = time.monotonic()
            try:
                action = policy.act(PolicyInput(config.instruction, observation, step))
            finally:
                exchange = getattr(policy, "last_exchange", None)
                if exchange:
                    recorder.event({"event": "model_exchange", "step": step, "exchange": exchange})
            policy_time_s = time.monotonic() - t0
            validate_action(action, config)
            if time.monotonic() - start >= config.max_wall_time_s:
                reason = "wall_time_limit"
                break
            if action.request_stop:
                reason = "policy_requested_stop"
                break
            robot.apply(action)
            backend.step(action.duration_s)
            completed_steps = step
            feedback = robot.feedback()
            observation = observe()
            assessment = (
                Assessment("not_evaluated", "Execution rejected or failed")
                if feedback.status in {"error", "rejected"}
                else task.evaluate(backend.evaluation_state(), feedback)
            )
            if dry_run and assessment.status != "not_evaluated":
                assessment = Assessment("not_evaluated", "Rejected task score during dry-run")
                raise ValueError("Dry-run must never produce a task score")
            recorder.event({"event": "transition", "step": step,
                            "action": asdict(action), "policy_time_s": policy_time_s,
                            "observation": recorder.observation(observation, step),
                            "evaluation": asdict(assessment)})
            if feedback.status in {"error", "rejected"}:
                status = "error"
                reason = "execution_error"
                break
            if assessment.status in {"succeeded", "failed"}:
                reason = "task_terminal"
                break
        if dry_run and status == "finished":
            status = "dry_run_complete"
    except NotImplementedError as exc:
        status, reason, error = "implementation_missing", "stub_reached", str(exc)
    except Exception as exc:
        status, reason, error = "error", "exception", f"{type(exc).__name__}: {exc}"
    finally:
        try:
            backend.close()
        except Exception as exc:
            status, reason = "error", "cleanup_error"
            error = (error + "; " if error else "") + f"close: {type(exc).__name__}: {exc}"
        evaluated = not dry_run and status == "finished" and assessment.status in {"succeeded", "failed"}
        result = {
            "task_id": config.task_id, "mode": mode, "status": status,
            "termination_reason": reason, "completed_steps": completed_steps,
            "wall_time_s": time.monotonic() - start,
            "task_evaluated": evaluated,
            "success": (assessment.status == "succeeded") if evaluated else None,
            "assessment": asdict(assessment), "error": error,
            "video_status": "not_implemented",
        }
        recorder.finish(result)
    return recorder.directory, result
