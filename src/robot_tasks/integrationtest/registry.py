"""Single catalogue used by the CLI, worker and debugger.

Entry points are trusted code registrations, never supplied by TOML. Importing
the catalogue does not import any check implementation or start Isaac Sim.
"""

from collections.abc import Callable
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path

from ..config import TaskConfig, load_config
from ..paths import PROJECT_ROOT
from ..robots.audit import require_cartesian_sim_ready, require_joint_sim_ready
from ..robots.profile import RobotProfile


@dataclass(frozen=True)
class CheckSpec:
    name: str
    description: str
    task_ids: tuple[str, ...]
    default_config: Path
    entry_point: str
    output_prefix: str
    preflight: Callable[[RobotProfile], object] | None
    debug_config: Path | None = None
    uses_model: bool = False
    uses_simulation: bool = True

    def load_config(self, path: Path | None = None, *, debug: bool = False) -> TaskConfig:
        default = (self.debug_config or self.default_config) if debug else self.default_config
        config = load_config(path if path is not None else default)
        if config.task_id not in self.task_ids:
            raise ValueError(f"Check/config mismatch: {self.name} expects {self.task_ids}, got {config.task_id}")
        return config

    def validate_options(self, *, gui: bool = False, model_config: Path | None = None) -> None:
        if gui and not self.uses_simulation:
            raise ValueError(f"{self.name}: --gui requires a simulation check")
        if model_config is not None and not self.uses_model:
            raise ValueError(f"{self.name}: --model-config is not used by this check")

    def run(self, config_path: Path, output_root: Path, *, gui: bool = False,
            model_config: Path | None = None, run_directory: Path | None = None) -> tuple[Path, dict]:
        self.validate_options(gui=gui, model_config=model_config)
        config = self.load_config(config_path)
        if self.preflight is not None:
            self.preflight(config.robot)
        module, function = self.entry_point.split(":")
        execute = getattr(import_module(f"{__package__}.{module}"), function)
        options = {"gui": gui, "run_directory": run_directory}
        if model_config is not None:
            options["model_path"] = model_config
        return execute(config.source, output_root, **options)


_CONFIGS = PROJECT_ROOT / "configs/checks"

CHECKS = {
    "gpt-pick-place": CheckSpec(
        "gpt-pick-place", "GPT 单臂抓放任务的全链路检查", ("GPT-PICK-PLACE",),
        PROJECT_ROOT / "configs/tasks/gpt_single_arm_pick_place.toml",
        "gpt_pick_place_check:run_gpt_pick_place_check", "R5_gpt_pick_place_",
        require_cartesian_sim_ready, uses_model=True,
    ),
    "joint": CheckSpec(
        "joint", "R5 关节与夹爪", ("R5-MOTION", "R5-MOTION-DEBUG"),
        _CONFIGS / "r5_motion.toml", "motion_check:run_motion_check", "R5_motion_",
        require_joint_sim_ready, _CONFIGS / "r5_motion_debug.toml",
    ),
    "cartesian": CheckSpec(
        "cartesian", "TCP、IK 与不可达目标拒绝", ("R5-CARTESIAN", "R5-CARTESIAN-DEBUG"),
        _CONFIGS / "r5_cartesian.toml", "cartesian_check:run_cartesian_check", "R5_cartesian_",
        require_cartesian_sim_ready, _CONFIGS / "r5_cartesian_debug.toml",
    ),
    "interaction": CheckSpec(
        "interaction", "脚本方块抓放", ("R5-INTERACTION",),
        _CONFIGS / "r5_interaction.toml", "interaction_check:run_interaction_check", "R5_interaction_",
        require_cartesian_sim_ready, _CONFIGS / "r5_interaction_debug.toml",
    ),
    "gpt": CheckSpec(
        "gpt", "真实 GPT、三路 RGB、R5 动作与反馈", ("R5-GPT",),
        _CONFIGS / "r5_gpt.toml", "gpt_check:run_gpt_check", "R5_gpt_",
        require_cartesian_sim_ready, _CONFIGS / "r5_gpt_debug.toml", uses_model=True,
    ),
    "cameras": CheckSpec(
        "cameras", "三路 RGB 与腕部跟随", ("R5-CAMERAS",),
        _CONFIGS / "r5_cameras.toml", "camera_check:run_camera_check", "R5_cameras_",
        require_cartesian_sim_ready,
    ),
    "api": CheckSpec(
        "api", "一次真实模型图像/API 检查，不启动仿真", ("R5-GPT",),
        _CONFIGS / "r5_gpt.toml", "gpt_api_check:run_gpt_api_check", "GPT_api_",
        None, uses_model=True, uses_simulation=False,
    ),
}


def get_check(name: str) -> CheckSpec:
    try:
        return CHECKS[name]
    except KeyError:
        raise ValueError(f"Unknown check: {name}; available: {', '.join(CHECKS)}") from None
