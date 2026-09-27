"""Benchmark task catalogue: identity, implementation and default experiment."""

from dataclasses import dataclass
from pathlib import Path

from .config import TaskConfig, load_config
from .paths import PROJECT_ROOT
from .tasks.cylinder_insertion import CylinderInsertion
from .tasks.dual_arm_handover import DualArmHandover


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    name: str
    factory: type
    arm_count: int
    implemented: bool
    evidence: str
    default_config: Path


TASKS = {
    "L1-01": TaskSpec(
        "L1-01", "圆柱插孔", CylinderInsertion, 1, False,
        "已做重力落入/碰撞基础实验；完整机械臂任务未实现",
        PROJECT_ROOT / "configs/tasks/l1_01_cylinder_insertion.toml",
    ),
    "L2-02": TaskSpec(
        "L2-02", "双臂夹持力交接", DualArmHandover, 2, False,
        "仅有参考任务调研；本地双臂交接未验证",
        PROJECT_ROOT / "configs/tasks/l2_02_dual_arm_handover.toml",
    ),
}


def get_spec(config: TaskConfig) -> TaskSpec:
    if config.task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {config.task_id}")
    spec = TASKS[config.task_id]
    if len(config.arms) != spec.arm_count:
        raise ValueError(f"{spec.task_id} requires {spec.arm_count} arm(s)")
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
