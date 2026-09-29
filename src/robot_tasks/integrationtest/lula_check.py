"""Check real NVIDIA Lula against the R5 model without starting Isaac or GPT."""

import argparse
from dataclasses import asdict
import importlib
import importlib.metadata
import os
from pathlib import Path
import sys
import time

import numpy as np

from ..config import load_config
from ..backends.settings import SimulationSettings
from ..recording import write_json
from ..robots.lula import R5LulaKinematics


_dll_directories = []  # Keep Windows search handles alive for native dependencies.


def load_native_lula():
    """Offline checks bypass Kit startup, using the SAME bundled native library.

    Runtime simulation enables the extension through Kit instead. No downloads,
    PATH configuration files or alternative IK implementation are introduced.
    """
    try:
        return importlib.import_module("lula")
    except ModuleNotFoundError as exc:
        if exc.name != "lula":
            raise
    distribution = importlib.metadata.distribution("isaacsim")
    bundle = Path(distribution.locate_file("isaacsim/exts/isaacsim.robot_motion.lula/pip_prebundle"))
    if not bundle.is_dir():
        raise RuntimeError("Isaac Sim's bundled Lula was not found; run with the Isaac Python environment")
    if os.name == "nt":
        _dll_directories.append(os.add_dll_directory(str(bundle)))
    sys.path.insert(0, str(bundle))
    return importlib.import_module("lula")


def check(config, *, samples=30):
    kin = R5LulaKinematics(config.robot, SimulationSettings.from_config(config).asset_directory,
                           sampling_seed=config.seed, lula_module=load_native_lula())
    rng = np.random.default_rng(config.seed)
    cases = []
    for mount in config.robot.mounts:
        home = np.asarray(mount.home_positions_rad)
        for index in range(samples):
            q = np.clip(home + rng.uniform(-.35, .35, len(home)), kin.lower, kin.upper)
            for frame in ("world", "robot_base"):
                target = kin.forward(q, mount.name, frame)
                start = time.perf_counter()
                result = kin.solve(target, mount.name, home, frame)
                cases.append({"arm": mount.name, "sample": index, "frame": frame,
                              "solve_time_s": time.perf_counter() - start, **asdict(result)})
    unreachable = kin.solve((3., 0., .5, 1., 0., 0., 0.), config.robot.mounts[0].name,
                            config.robot.mounts[0].home_positions_rad)
    return {"scope": "Native Lula kinematics only; no physics, GPT calls, or task-success measurement",
            "solver": kin.metadata, "cases": cases, "unreachable": asdict(unreachable),
            "passed": all(c["converged"] for c in cases) and not unreachable.converged}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/tasks/gpt_single_arm_pick_place.toml"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = check(load_config(args.config))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, result)
    print(f"Lula native check: passed={result['passed']}; cases={len(result['cases'])}; output={args.output}")
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
