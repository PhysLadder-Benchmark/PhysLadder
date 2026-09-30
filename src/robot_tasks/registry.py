"""Discover maintained TOMLs and bind only explicitly trusted task implementations."""

import tomllib
from dataclasses import dataclass
from pathlib import Path

from .config import TaskConfig, load_config
from .paths import PROJECT_ROOT
from .tasks.bench.spec import build_spec
from .tasks.bench.task import BenchTask
from .tasks.pick_place import SingleArmPickPlace
from .tasks.screw.task import ScrewTask
from .tasks.uncap.task import UncapTask


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    name: str
    factory: type | None
    active_arm_counts: tuple[int, ...]
    implemented: bool
    evidence: str
    default_config: Path


def catalogue(directory: Path) -> dict[str, TaskSpec]:
    result = {}
    factories = {"household": BenchTask, "uncap": UncapTask, "screw": ScrewTask}
    for path in sorted(directory.glob("*.toml")):
        if path.stem.endswith(".local"):
            continue  # Local experiment overrides are selected with --config.
        with path.open("rb") as stream:
            data = tomllib.load(stream)
        task = data["task"]
        task_id = task["id"]
        if task_id in result:
            raise ValueError(f"Duplicate task ID: {task_id}")
        if task.get("simulation_enabled") is False:
            result[task_id] = TaskSpec(
                task_id, task["name"], None, (), False, "real_only", path
            )
            continue
        key = task.get("implementation")
        if key not in factories:
            raise ValueError(
                f"{path.name}: unknown trusted task implementation {key!r}"
            )
        arms = len(data["robot"]["active_arms"])
        evidence = task.get(
            "simulation_limitations",
            "Local physical scene; independent measured scoring",
        )
        if key == "household":
            design = build_spec(task_id)
            if tuple(data["robot"]["active_arms"]) != design.arms:
                raise ValueError(
                    f"{task_id}: configured arms do not match physical task"
                )
            evidence = design.mechanism + "; " + design.limitations
        implemented = task.get("status") in {"implemented", "implemented_proxy"}
        result[task_id] = TaskSpec(
            task_id, task["name"], factories[key], (arms,), implemented, evidence, path
        )
    return result


TASKS = catalogue(PROJECT_ROOT / "configs/tasks")
# The small engineering check is kept separate from the benchmark's task definitions.
TASKS["GPT-PICK-PLACE"] = TaskSpec(
    "GPT-PICK-PLACE",
    "GPT 单臂抓放工程检查",
    SingleArmPickPlace,
    (1,),
    True,
    "Engineering integration check, not part of the L1-L3 benchmark pool",
    PROJECT_ROOT / "configs/checks/gpt_single_arm_pick_place.toml",
)


def get_spec(config: TaskConfig) -> TaskSpec:
    if config.task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {config.task_id}")
    spec = TASKS[config.task_id]
    if len(config.arms) not in spec.active_arm_counts:
        raise ValueError(
            f"{spec.task_id} permits active arm counts {spec.active_arm_counts}"
        )
    return spec


def resolve_task(
    task_id: str | None = None, config_path: Path | None = None
) -> tuple[TaskSpec, TaskConfig]:
    if task_id is not None and task_id not in TASKS:
        raise ValueError(f"Unknown task ID: {task_id}; available: {', '.join(TASKS)}")
    if task_id is None and config_path is None:
        raise ValueError("Provide a task ID or --config; use 'list' to see tasks")
    config = load_config(
        config_path if config_path is not None else TASKS[task_id].default_config
    )
    if task_id is not None and config.task_id != task_id:
        raise ValueError(
            f"Task/config mismatch: selected {task_id}, config declares {config.task_id}"
        )
    return get_spec(config), config
