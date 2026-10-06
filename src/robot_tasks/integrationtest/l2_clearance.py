"""Sampled R5/prop clearance, not a robot policy or continuous path certificate."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .bottle_clearance import Geometry, box_vertices, separation
from ..paths import PROJECT_ROOT
from ..registry import resolve_task
from ..robots.transforms import origin_transform, pose_residual, pose_transform
from ..tasks.bench.spec import build_spec


def solve_clear(geometry, goal, seed, width, obstacles):
    """Reject the algebraic elbow-down solution that passes through the table."""
    from scipy.optimize import least_squares

    def residual(q):
        error = pose_residual(pose_transform(geometry.kin.forward(q, "left")), goal)
        error[3:] *= .2
        return error

    rng = np.random.default_rng(0)
    for guess in (seed, *rng.uniform(geometry.kin.lower, geometry.kin.upper, (24, 6))):
        solution = least_squares(residual, guess, bounds=(geometry.kin.lower, geometry.kin.upper), max_nfev=150)
        if np.linalg.norm(residual(solution.x)) >= 1e-5:
            continue
        boxes, _ = geometry.boxes("left", solution.x, width)
        table_margin = min(float(b[:, 2].min()) for name, b in boxes.items() if name != "base_link")
        if table_margin < .001:
            continue
        margins = [(link, name, separation(shape, obstacle))
                   for link, shape in boxes.items() for name, obstacle in obstacles]
        worst = min(margins, key=lambda item: item[2])
        if worst[2] >= .001:
            return solution.x, worst, table_margin
    raise RuntimeError("No sampled IK solution with whole-arm table/fixture clearance")


def check(task):
    _, config = resolve_task(task)
    spec = build_spec(task)
    geometry = Geometry(config)
    samples = []
    article_frame = pose_transform(spec.design.articles[0].initial)
    article_yaw = math.atan2(article_frame[1, 0], article_frame[0, 0])

    def placed(local):
        return article_frame[:3, 3] + article_frame[:3, :3] @ local

    if task == "L2-03":
        for x in np.linspace(-.094, -.034, 7):
            samples.append(("approach", placed((x, 0, -.002)), article_yaw, .070))
        for x in np.linspace(-.034, -.134, 11):
            samples.append(("extract", placed((x, 0, -.002)), article_yaw, .045))
    elif task == "L2-07":
        # Grip the exposed front portion, 10 mm ahead of the handle centre,
        # so the rear of the fingers remains clear of the cabinet roof.
        for x in np.linspace(-.195, -.135, 7):
            samples.append(("approach", placed((x, 0, .053)), article_yaw, .070))
        for x in np.linspace(-.135, -.245, 12):
            samples.append(("pull", placed((x, 0, .053)), article_yaw, .040))
    else:
        raise ValueError(f"No clearance diagnostic for {task}")
    fixed = [(p.name, box_vertices(origin_transform(p.position, p.rotation),
                                  -np.array(p.size) / 2, np.array(p.size) / 2))
             for p in spec.design.fixtures if p.collision]
    q = np.array(next(m for m in config.robot.mounts if m.name == "left").home_positions_rad)
    report = []
    for phase, position, yaw, width in samples:
        obstacles = list(fixed)
        last_error = None
        for pitch in (math.pi / 2, math.pi / 3, math.pi / 4, math.pi / 6, 0.):
            try:
                q, worst, table_margin = solve_clear(geometry, origin_transform(position, (0, pitch, yaw)), q, width, obstacles)
                break
            except RuntimeError as exc:
                last_error = exc
        else:
            raise RuntimeError(f"{task} {phase} at {position}: {last_error}")
        report.append({"phase": phase, "tcp_position": list(position), "joints": q.tolist(),
                       "tool_pitch_deg": math.degrees(pitch), "tool_yaw_deg": math.degrees(yaw), "worst_pair": worst[:2],
                       "clearance_m": worst[2], "table_clearance_m": table_margin})
    return {"task": task, "passed": True, "samples": report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runs/l2_layout_clearance.json")
    output = parser.parse_args().output
    output.parent.mkdir(parents=True, exist_ok=True)
    results = []
    for task in ("L2-03", "L2-07"):
        try:
            result = check(task)
        except Exception as exc:
            result = {"task": task, "passed": False, "error": str(exc)}
        results.append(result)
        print(task, result["passed"], result.get("error", ""), flush=True)
    output.write_text(json.dumps({"scope": "Sampled arm/camera/bracket bounds versus table and fixtures plus IK. Not self/inter-arm collision, contact stability, continuous planning or GPT success.",
                                 "results": results}, indent=2), encoding="utf-8")
    return 0 if all(r["passed"] for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
