"""Sampled short-screw grasp/turn clearance; no robot control or model calls.

Uses the existing CAD/URDF box diagnostic. Not a continuous-path, self-collision,
grip-stability or physical thread-engagement test; never supplies poses to GPT.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

import numpy as np

from .bottle_clearance import Geometry, box_vertices
from .l2_clearance import solve_clear
from ..paths import PROJECT_ROOT
from ..registry import resolve_task
from ..robots.transforms import origin_transform
from ..tasks.screw.spec import SPEC


def check():
    _, config = resolve_task("L3-05")
    geometry = Geometry(config)
    s = SPEC
    head_grip_height = s.shaft_length + s.head_height / 2
    # A round head permits choosing the initial grasp yaw. Centre the planned
    # half-turn around zero instead of forcing a wrist to its +/-pi limit.
    start_yaw = s.required_turn_rad / 2
    fixture = box_vertices(origin_transform((*s.socket_xy, 0), (0, 0, 0)),
                           (-.040, -.040, 0), (.040, .040, s.socket_rim))
    obstacles = [("socket_enclosing_box", fixture)]
    q = np.array(next(m for m in config.robot.mounts if m.name == "left").home_positions_rad)
    candidates = []
    for lift in (.06, .03, 0.):
        candidates.append(("source_approach", (*s.source_xy, s.initial_pose[2] + head_grip_height + lift),
                           start_yaw, .070))
    candidates.append(("source_grasp", (*s.source_xy, s.initial_pose[2] + head_grip_height),
                       start_yaw, 2 * s.head_radius))
    for turn in np.linspace(0, -s.required_turn_rad, 13):
        tip_z = s.socket_rim - .010 + s.pitch * turn / (2 * math.pi)
        candidates.append(("nominal_held_turn", (*s.socket_xy, tip_z + head_grip_height),
                           start_yaw + turn, 2 * s.head_radius))
    candidates.append(("release", candidates[-1][1], candidates[-1][2], .070))
    samples = []
    for phase, position, yaw, width in candidates:
        try:
            q, worst, table = solve_clear(geometry, origin_transform(position, (0, math.pi / 2, yaw)),
                                         q, width, obstacles)
        except RuntimeError as exc:
            raise RuntimeError(f"{phase} at {position}, yaw={yaw}: {exc}") from exc
        samples.append({"phase": phase, "tcp_position": list(position), "yaw_rad": float(yaw),
                        "width_m": width, "joints": q.tolist(), "fixture_clearance_m": worst[2],
                        "worst_pair": worst[:2], "table_clearance_m": table})
    return {"passed": True, "spec": asdict(s), "gpt_called": False,
            "scope": "Sampled joint-limit-aware IK and conservative arm/finger/camera/bracket boxes vs table/socket. "
                     "Not continuous paths, self/inter-arm collision, physical engagement or GPT success.",
            "samples": samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runs/screw_short_clearance.json")
    output = parser.parse_args().output
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = check()
    except Exception as exc:
        result = {"passed": False, "task_version": SPEC.version, "error": str(exc), "gpt_called": False}
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
