"""Load a portable TOML episode configuration without simulator dependencies."""

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path

from .robots.profile import RobotProfile, load_robot_profile, names


@dataclass(frozen=True)
class TaskConfig:
    source: Path
    raw: dict
    task_id: str
    instruction: str
    robot: RobotProfile
    arms: tuple[str, ...]
    cameras: tuple[str, ...]
    seed: int
    max_steps: int
    max_wall_time_s: float
    control_dt_s: float
    physics_dt_s: float
    include_robot_state: bool
    include_feedback: bool
    save_rgb: bool

    @property
    def robot_model(self) -> str:
        return self.robot.model

    @property
    def asset_path(self) -> Path:
        return self.robot.asset_path


@dataclass(frozen=True)
class ExecutionSettings:
    """Control ticks, not model calls or raw physics substeps."""

    max_action_steps: int = 1
    completion_samples: int = 1
    wait_steps: int = 1
    settle_steps: int = 0
    stop_steps: int = 0
    max_decisions: int | None = None
    gripper_timing: str = "simultaneous"

    @classmethod
    def from_config(cls, config: TaskConfig) -> "ExecutionSettings":
        data = dict(config.raw.get("execution", {}))
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"Unknown execution settings: {sorted(unknown)}")
        result = cls(**data)
        if result.gripper_timing not in {"simultaneous", "after_motion"}:
            raise ValueError("execution.gripper_timing must be simultaneous or after_motion")
        for name in ("max_action_steps", "completion_samples", "wait_steps", "settle_steps", "stop_steps"):
            value = getattr(result, name)
            minimum = 0 if name in {"settle_steps", "stop_steps"} else 1
            if type(value) is not int or not minimum <= value <= config.max_steps:
                raise ValueError(f"execution.{name} must be an integer in [{minimum}, max_steps]")
        if result.completion_samples > result.max_action_steps:
            raise ValueError("completion_samples cannot exceed max_action_steps")
        if result.settle_steps >= config.max_steps:
            raise ValueError("settle_steps must leave an action budget")
        if result.max_decisions is not None and (type(result.max_decisions) is not int
                                               or not 1 <= result.max_decisions <= 1000):
            raise ValueError("execution.max_decisions must be in [1, 1000]")
        return result


def _names(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value or any(
        not isinstance(v, str) or not v.replace("_", "").isalnum() for v in value
    ):
        raise ValueError(f"{label} must be a nonempty list of simple names")
    if len(set(value)) != len(value):
        raise ValueError(f"{label} contains duplicate names")
    return tuple(value)


def _positive(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return float(value)


def _boolean(value: object, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be true or false")
    return value


def load_config(path: Path) -> TaskConfig:
    source = path.expanduser().resolve()
    with source.open("rb") as f:
        raw = tomllib.load(f)
    if "planner" in raw:
        raise ValueError("Only built-in R5 IK is supported; remove the obsolete [planner] section")
    required = {"schema_version", "task", "robot", "runtime", "observation", "recording"}
    if not required.issubset(raw):
        raise ValueError(f"Missing config sections: {sorted(required - raw.keys())}")
    if type(raw["schema_version"]) is not int or raw["schema_version"] != 2:
        raise ValueError("Use schema_version = 2 with an R5 robot.profile and active_arms; legacy robot configs are not supported")
    task, robot = raw["task"], raw["robot"]
    runtime, observation = raw["runtime"], raw["observation"]
    for label, value in (("task.id", task["id"]), ("task.instruction", task["instruction"])):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{label} must be a nonempty string")
    if not isinstance(robot, dict) or set(robot) != {"profile", "active_arms"}:
        raise ValueError("robot must contain only profile and active_arms; this project uses R5 exclusively")
    if not isinstance(robot["profile"], str) or not robot["profile"].strip():
        raise ValueError("robot.profile must reference an R5 profile TOML")
    profile = load_robot_profile(source.parent / robot["profile"])
    arms = names(robot["active_arms"], "robot.active_arms")
    if not set(arms).issubset(profile.available_arms):
        raise ValueError("robot.active_arms must be a nonempty subset of the R5 left/right arms")
    max_steps, seed = runtime["max_steps"], runtime["seed"]
    if type(max_steps) is not int or max_steps <= 0 or type(seed) is not int or seed < 0:
        raise ValueError("max_steps must be a positive integer; seed a nonnegative integer")
    control_dt = _positive(runtime["control_dt_s"], "control_dt_s")
    physics_dt = _positive(runtime["physics_dt_s"], "physics_dt_s")
    ratio = control_dt / physics_dt
    if not math.isclose(ratio, round(ratio), abs_tol=1e-6) or ratio < 1:
        raise ValueError("control_dt_s must be an integer multiple of physics_dt_s")
    if "save_video" in raw["recording"]:
        _boolean(raw["recording"]["save_video"], "save_video")
    return TaskConfig(
        source=source, raw=raw, task_id=task["id"], instruction=task["instruction"],
        robot=profile, arms=arms, cameras=_names(observation["cameras"], "cameras"),
        seed=seed, max_steps=max_steps,
        max_wall_time_s=_positive(runtime["max_wall_time_s"], "max_wall_time_s"),
        control_dt_s=control_dt, physics_dt_s=physics_dt,
        include_robot_state=_boolean(observation["include_robot_state"], "include_robot_state"),
        include_feedback=_boolean(observation["include_feedback"], "include_feedback"),
        save_rgb=_boolean(raw["recording"]["save_rgb"], "save_rgb"),
    )
