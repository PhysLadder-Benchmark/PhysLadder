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
    if task == "L2-03":
        for x in np.linspace(.241, .301, 7):
            samples.append(("approach", (x, .34, .064), 0., .070, {}))
        for x in np.linspace(.301, .201, 11):
            samples.append(("extract", (x, .34, .064), 0., .045, {}))
    elif task == "L2-07":
        # Grip the exposed front portion, 10 mm ahead of the handle centre,
        # so the rear of the fingers remains clear of the cabinet roof.
        for x in np.linspace(.245, .305, 7):
            samples.append(("approach", (x, .34, .067), 0., .070, {}))
        for x in np.linspace(.305, .195, 12):
            samples.append(("pull", (x, .34, .067), 0., .040, {}))
    else:
        anchor = np.array(spec.parameters["hinge_anchor"])
        latch = np.array(spec.design.articles[1].initial[:3]) - [.007, 0, 0]
        for offset in np.linspace(.06, 0, 7):
            samples.append(("approach", latch - [offset, 0, 0], 0., .070, {}))
        for angle in np.linspace(0, math.radians(115), 24):
            rotation = origin_transform((0, 0, 0), (0, angle, 0))
            position = anchor + rotation[:3, :3] @ (latch - [.017, 0, 0] - anchor)
            lid = pose_transform(spec.design.articles[0].initial)
            lid[:3, 3] = anchor + rotation[:3, :3] @ (lid[:3, 3] - anchor)
            lid[:3, :3] = rotation[:3, :3] @ lid[:3, :3]
            # Closing axis remains across the latch width. Lid pitch does not
            # prescribe tool pitch; side-face contact can pivot during opening.
            samples.append(("open", position, 0., .040, {"lid": lid}))
    fixed = [(p.name, box_vertices(origin_transform(p.position, p.rotation),
                                  -np.array(p.size) / 2, np.array(p.size) / 2))
             for p in spec.design.fixtures if p.collision]
    q = np.array(next(m for m in config.robot.mounts if m.name == "left").home_positions_rad)
    report = []
    for phase, position, pitch, width, movable in samples:
        obstacles = list(fixed)
        for name, transform in movable.items():
            article = next(a for a in spec.design.articles if a.name == name)
            for part in article.parts:
                if part.collision:
                    obstacles.append((name + "/" + part.name,
                                      box_vertices(transform @ origin_transform(part.position, part.rotation),
                                                   -np.array(part.size) / 2, np.array(part.size) / 2)))
        last_error = None
        for pitch in (math.pi / 2, math.pi / 3, math.pi / 4, math.pi / 6, 0.):
            try:
                q, worst, table_margin = solve_clear(geometry, origin_transform(position, (0, pitch, 0)), q, width, obstacles)
                break
            except RuntimeError as exc:
                last_error = exc
        else:
            raise RuntimeError(f"{task} {phase} at {position}: {last_error}")
        report.append({"phase": phase, "tcp_position": list(position), "joints": q.tolist(),
                       "tool_pitch_deg": math.degrees(pitch), "worst_pair": worst[:2],
                       "clearance_m": worst[2], "table_clearance_m": table_margin})
    return {"task": task, "passed": True, "samples": report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runs/l2_layout_clearance.json")
    output = parser.parse_args().output
    output.parent.mkdir(parents=True, exist_ok=True)
    results = []
    for task in ("L2-03", "L2-05", "L2-07"):
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
