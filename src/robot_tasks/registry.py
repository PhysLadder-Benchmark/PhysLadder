"""Benchmark task catalogue: identity, implementation and default experiment."""

from dataclasses import dataclass
from pathlib import Path

from .config import TaskConfig, load_config
from .paths import PROJECT_ROOT
from .tasks.l1.task import L1Task
from .tasks.l1.specs import RIGID_IDS, FILES, build_design
from .tasks.dual_arm_handover import DualArmHandover
from .tasks.pick_place import SingleArmPickPlace


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    name: str
    factory: type
    active_arm_counts: tuple[int, ...]
    implemented: bool
    evidence: str
    default_config: Path


TASKS = {
    "GPT-PICK-PLACE": TaskSpec(
        "GPT-PICK-PLACE", "GPT 单臂方块抓放", SingleArmPickPlace, (1,), True,
        "GPT 自主选择动作；实测接触、抬起、释放与稳定放置独立评分",
        PROJECT_ROOT / "configs/tasks/gpt_single_arm_pick_place.toml",
    ),
    "L2-02": TaskSpec(
        "L2-02", "双臂夹持力交接", DualArmHandover, (2,), False,
        "仅有参考任务调研；本地双臂交接未验证",
        PROJECT_ROOT / "configs/tasks/l2_02_dual_arm_handover.toml",
    ),
}


for task_id, filename in zip(RIGID_IDS, FILES):
    TASKS[task_id] = TaskSpec(task_id, build_design(task_id).name, L1Task, (1,), True,
        "Shared GPT RGB-D workflow with right-arm observation; measured goal/process scoring; l1-rigid-v1",
        PROJECT_ROOT / f"configs/tasks/{task_id.lower().replace('-', '_')}_{filename}.toml")


def get_spec(config: TaskConfig) -> TaskSpec:
    if config.task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {config.task_id}")
    spec = TASKS[config.task_id]
    if len(config.arms) not in spec.active_arm_counts:
        raise ValueError(f"{spec.task_id} permits active arm counts {spec.active_arm_counts}")
    return spec


def resolve_task(task_id: str | None = None, config_path: Path | None = None) -> tuple[TaskSpec, TaskConfig]:
    """Resolve a named task, optionally overriding its default experiment.

    Config-only invocation remains supported. A named task may never silently
    switch implementation because an override refers to a different task ID.
    """
    if task_id is not None and task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {task_id}; available: {', '.join(TASKS)}")
    if task_id is None and config_path is None:
        raise ValueError("Provide a task ID or --config; use 'list' to see tasks")
    config = load_config(config_path if config_path is not None else TASKS[task_id].default_config)
    if task_id is not None and config.task_id != task_id:
        raise ValueError(f"Task/config mismatch: selected {task_id}, config declares {config.task_id}")
    return get_spec(config), config
