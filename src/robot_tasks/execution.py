"""Assemble a registered benchmark task and delegate its lifecycle to runner."""

from dataclasses import replace
from pathlib import Path

from .backends.isaacsim import IsaacSimBackend
from .backends.mock import MockBackend, MockRobot, MockTask, NoopPolicy
from .config import TaskConfig
from .paths import PROJECT_ROOT
from .policies.gpt import GPTPolicy
from .policies.model_config import load_gpt_settings
from .policies.scripted import ScriptedPolicy
from .registry import TaskSpec
from .robots.adapters import R5Adapter
from .robots.audit import require_cartesian_sim_ready
from .runner import run_episode


def _gpt_policy(config: TaskConfig, model_config: Path | None):
    return GPTPolicy(config, load_gpt_settings(model_config or PROJECT_ROOT / "configs/models/gpt.toml"))


def _scripted_policy(config: TaskConfig, model_config: Path | None):
    if model_config is not None:
        raise ValueError("--model-config is only used with --policy gpt")
    return ScriptedPolicy()


POLICIES = {"gpt": _gpt_policy, "scripted": _scripted_policy}


def execute_task(spec: TaskSpec, config: TaskConfig, output_root: Path, *, policy_name: str = "gpt",
                 model_config: Path | None = None, gui: bool = False, dry_run: bool = False,
                 step_limit: int | None = None) -> tuple[Path, dict]:
    if config.task_id != spec.task_id:
        raise ValueError("Task/config mismatch")
    if dry_run:
        return run_episode(config, MockBackend(), MockRobot(), MockTask(), NoopPolicy(config.control_dt_s),
                           output_root, dry_run=True, step_limit=step_limit)
    if not spec.implemented:
        raise ValueError(f"{spec.task_id} is not implemented: {spec.evidence}. Use dry-run for the scaffold.")
    if policy_name not in POLICIES:
        raise ValueError(f"Unknown policy: {policy_name}")
    require_cartesian_sim_ready(config.robot)
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {
            **config.raw.get("simulation", {}), "headless": False,
        }})
    policy = POLICIES[policy_name](config, model_config)
    return run_episode(config, IsaacSimBackend(), R5Adapter(), spec.factory(), policy, output_root)
