"""Project-local renderer settings; no driver or installed Kit files are changed."""
import os


def simulation_config(width=1024, height=768):
    backend = os.environ.get("SIM_GRAPHICS_BACKEND", "d3d12")
    if backend not in {"d3d12", "vulkan"}:
        raise ValueError(f"Unsupported graphics backend: {backend}")
    return {
        "headless": True,
        "width": width,
        "height": height,
        "active_gpu": 0,
        "physics_gpu": 0,
        "multi_gpu": False,
        "extra_args": [f"--/app/vulkan={'true' if backend == 'vulkan' else 'false'}"],
    }
