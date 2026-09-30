"""Explicit simulation settings, loadable without importing Isaac Sim."""

from dataclasses import dataclass
import math
import os
from pathlib import Path

from ..config import TaskConfig


@dataclass(frozen=True)
class SimulationSettings:
    asset_directory: Path
    headless: bool = True
    graphics_backend: str = "d3d12" if os.name == "nt" else "vulkan"
    width: int = 960
    height: int = 720
    fps: int = 12
    arm_stiffness: float = 1500.0
    arm_damping: float = 80.0
    finger_stiffness: float = 2000.0
    finger_damping: float = 40.0
    self_collision: bool = False
    solver_position_iterations: int = 32
    solver_velocity_iterations: int = 0
    camera_position: tuple[float, ...] = (1.6, 0.0, 1.4)
    camera_target: tuple[float, ...] = (0.28, 0.0, 0.2)
    camera_profile: str = ""  # Relative to episode config; empty keeps legacy overview.
    capture_render_passes: int = 2
    environment: str = "plain"
    robot_appearance: str = "vendor_white"

    @classmethod
    def from_config(cls, config: TaskConfig) -> "SimulationSettings":
        data = dict(config.raw.get("simulation", {}))
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"Unknown simulation settings: {sorted(unknown)}")
        directory = data.pop("asset_directory", "../../build/r5")
        if not isinstance(directory, str) or not directory:
            raise ValueError("simulation.asset_directory must be a path")
        result = cls(asset_directory=(config.source.parent / directory).resolve(), **data)
        if result.environment not in {"plain", "lab_room_v1"}:
            raise ValueError("simulation.environment must be plain or lab_room_v1")
        if result.robot_appearance not in {"vendor_white", "r5_black_silver_v1"}:
            raise ValueError("simulation.robot_appearance must be vendor_white or r5_black_silver_v1")
        if not isinstance(result.camera_profile, str):
            raise ValueError("simulation.camera_profile must be a path string")
        if type(result.capture_render_passes) is not int or not 1 <= result.capture_render_passes <= 8:
            raise ValueError("capture_render_passes must be in [1, 8]")
        if result.graphics_backend not in {"vulkan", "d3d12"}:
            raise ValueError("graphics_backend must be vulkan or d3d12")
        for name in ("headless", "self_collision"):
            if type(getattr(result, name)) is not bool:
                raise ValueError(f"simulation.{name} must be boolean")
        for name in ("width", "height", "fps"):
            value = getattr(result, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"simulation.{name} must be a positive integer")
        for name, minimum in (("solver_position_iterations", 1), ("solver_velocity_iterations", 0)):
            value = getattr(result, name)
            if type(value) is not int or not minimum <= value <= 255:
                raise ValueError(f"simulation.{name} must be an integer in [{minimum}, 255]")
        for name in ("arm_stiffness", "arm_damping", "finger_stiffness", "finger_damping"):
            value = getattr(result, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"simulation.{name} must be finite and positive")
        for name in ("camera_position", "camera_target"):
            value = getattr(result, name)
            if not isinstance(value, (tuple, list)) or len(value) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value
            ):
                raise ValueError(f"simulation.{name} must contain three finite numbers")
        return result

    def application_config(self) -> dict:
        return {
            "headless": self.headless, "width": self.width, "height": self.height,
            "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False,
            # Kit runs in an isolated worker; fast shutdown avoids Windows DLL teardown crashes.
            "fast_shutdown": True,
            "extra_args": [f"--/app/vulkan={'true' if self.graphics_backend == 'vulkan' else 'false'}"],
        }
