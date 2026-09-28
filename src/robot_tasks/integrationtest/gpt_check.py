"""Configurable GPT motion fixture, using the same runner as real tasks."""

from dataclasses import dataclass, replace
import math
from pathlib import Path

from ..backends.isaacsim import IsaacSimBackend
from ..config import ExecutionSettings, load_config
from ..policies.gpt import GPTPolicy
from ..policies.model_config import load_gpt_settings, read_api_key
from ..robots.adapters import R5Adapter
from ..robots.audit import require_cartesian_sim_ready
from ..robots.transforms import pose_errors
from ..runner import run_episode
from ..tasks.pick_place_scene import InteractionScene, InteractionSettings
from ..types import Assessment


@dataclass(frozen=True)
class MotionGoal:
    arm: str
    lift_delta_m: float
    gripper_width_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    gripper_tolerance_m: float
    inactive_joint_tolerance_rad: float

    @classmethod
    def from_config(cls, config):
        goal = cls(**config.raw["motion_goal"])
        if goal.arm not in config.arms:
            raise ValueError("motion_goal.arm must be an active R5 arm")
        for name in cls.__dataclass_fields__:
            if name == "arm":
                continue
            value = getattr(goal, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"motion_goal.{name} must be finite and positive")
        lo, hi = config.robot.gripper.width_limits_m
        if not lo <= goal.gripper_width_m <= hi:
            raise ValueError("motion_goal.gripper_width_m is outside the robot limits")
        return goal

    def instruction(self, template):
        return template.format(arm=self.arm, lift_delta_m=self.lift_delta_m, gripper_width_m=self.gripper_width_m)


class MotionCheckTask:
    """This evaluator belongs to the fixture; the runner knows no lift target."""

    def __init__(self, goal):
        self.goal = goal

    def build_scene(self, backend, config):
        self.scene = InteractionScene(backend, config, InteractionSettings.from_config(config))

    def reset(self, backend, seed):
        self.scene.reset(self.goal.arm)
        self.initial = self.target = None
        self.moved = self.opened = False
        self.drift = 0.0

    def evaluate(self, state, feedback):
        robot = state.values["robot"]
        active = robot[self.goal.arm]
        if self.initial is None:
            self.initial = robot
            self.target = list(active.tcp_pose_world_wxyz)
            self.target[2] += self.goal.lift_delta_m
        p_error, r_error = pose_errors(active.tcp_pose_world_wxyz, self.target)
        self.moved = self.moved or (p_error <= self.goal.position_tolerance_m
                                    and r_error <= self.goal.orientation_tolerance_rad)
        self.opened = self.opened or (self.moved and abs(active.gripper_width_m - self.goal.gripper_width_m)
                                     <= self.goal.gripper_tolerance_m)
        self.drift = max(self.drift, max((abs(q - original) for name, arm in robot.items() if name != self.goal.arm
                                        for q, original in zip(arm.joint_positions_rad,
                                                                self.initial[name].joint_positions_rad)), default=0.0))
        passed = self.moved and self.opened and self.drift <= self.goal.inactive_joint_tolerance_rad
        return Assessment("running" if self.drift <= self.goal.inactive_joint_tolerance_rad else "failed",
                          "Awaiting model stop after the configured motion checks",
                          {"tcp_lift_reached": self.moved, "gripper_width_reached": self.opened,
                           "inactive_joint_drift_rad": self.drift, "motion_goal_met": passed})


def run_gpt_check(config_path: Path, output_root: Path, *, gui=False, run_directory=None,
                  model_path: Path | None = None):
    config = load_config(config_path)
    require_cartesian_sim_ready(config.robot)
    goal = MotionGoal.from_config(config)
    config = replace(config, instruction=goal.instruction(config.instruction))
    ExecutionSettings.from_config(config)
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    settings = load_gpt_settings(model_path or config.source.parent / config.raw["gpt_check"]["model_config"])
    read_api_key(settings.api_key_env)
    def summarize(result):
        checks = result["assessment"]["metrics"]
        passed = (result["status"] == "finished" and checks.get("motion_goal_met", False)
                  and result["termination_reason"] == "policy_requested_stop" and result["model_calls"] >= 2)
        result.update(status="completed" if result["status"] == "finished" else result["status"],
                      passed=passed, task_evaluated=False, success=None, task_success=None,
                      false_completion=bool(result["policy_requested_stop"] and not checks.get("motion_goal_met", False)),
                      scope="RGB/API/R5 motion integration only; not contact-task performance")
        result["assessment"] = {"status": "not_evaluated", "reason": result["scope"], "metrics": checks}
        return result

    return run_episode(config, IsaacSimBackend(), R5Adapter(), MotionCheckTask(goal),
                                    GPTPolicy(config, settings), output_root,
                                    run_directory=run_directory, record_video=True, summarize=summarize)
