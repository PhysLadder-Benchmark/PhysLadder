from dataclasses import dataclass

from .config import TaskConfig
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


TASKS = {
    "L1-01": TaskSpec(
        "L1-01", "圆柱插孔", CylinderInsertion, 1, False,
        "已做重力落入/碰撞基础实验；完整机械臂任务未实现",
    ),
    "L2-02": TaskSpec(
        "L2-02", "双臂夹持力交接", DualArmHandover, 2, False,
        "仅有参考任务调研；本地双臂交接未验证",
    ),
}


def get_spec(config: TaskConfig) -> TaskSpec:
    if config.task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {config.task_id}")
    spec = TASKS[config.task_id]
    if len(config.arms) != spec.arm_count:
        raise ValueError(f"{spec.task_id} requires {spec.arm_count} arm(s)")
    return spec
