"""Sampled native L2 reach/fixture checks; no model, robot script or success claim.

Run with the existing planner Python (numpy/scipy/trimesh). Whole-arm CAD boxes
are conservative; this is not continuous path or inter-arm collision certification.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from ..backends.scene_geometry import bench_geometry
from ..paths import PROJECT_ROOT
from ..registry import resolve_task
from ..robots.transforms import origin_transform, pose_residual, pose_transform
from ..tasks.bench.spec import build_spec
from .bottle_clearance import Geometry, box_vertices, separation


def solve_clear(geometry, goal, seed, width, obstacles, *, arm="left", table_z=0.0):
    from scipy.optimize import least_squares

    def residual(q):
        error = pose_residual(pose_transform(geometry.kin.forward(q, arm)), goal)
        error[3:] *= 0.2
        return error

    rng = np.random.default_rng(0)
    best = None
    for guess in (seed, *rng.uniform(geometry.kin.lower, geometry.kin.upper, (36, 6))):
        solution = least_squares(
            residual,
            guess,
            bounds=(geometry.kin.lower, geometry.kin.upper),
            max_nfev=180,
        )
        if np.linalg.norm(residual(solution.x)) >= 1e-5:
            continue
        boxes, _ = geometry.boxes(arm, solution.x, width)
        margin = min(
            float(b[:, 2].min()) - table_z
            for name, b in boxes.items()
            if name != "base_link"
        )
        margins = [
            (link, name, separation(shape, obstacle))
            for link, shape in boxes.items()
            for name, obstacle in obstacles
        ]
        worst = min(margins, key=lambda v: v[2]) if margins else ("none", "none", 1.0)
        candidate = (min(margin, worst[2]), margin, worst)
        if best is None or candidate[0] > best[0]:
            best = candidate
        if worst[2] >= 0.0005 and margin >= 0.0005:
            return solution.x, worst, margin
    raise RuntimeError(
        f"No sampled clear pose at {goal[:3, 3].tolist()}; best (minimum, table, fixture)={best}"
    )


def samples_and_fixtures(task, spec):
    p, z = spec.parameters, spec.parameters["table_z"]
    samples, fixed = [], []
    down_left, down_right = (0.0, math.pi / 2, math.pi), (0.0, math.pi / 2, 0.0)
    if task == "L2-07":
        x, y, h = spec.design.articles[0].initial[:3]
        samples = [
            ("left", (x, y + 0.112 + step, h + 0.026), (0.0, 0.3, -math.pi / 2), 0.040)
            for step in (0.0, 0.04, 0.08)
        ]
        t = pose_transform(p["native_instances"][0]["pose"])
        fixed.append(
            (
                "cabinet_envelope",
                box_vertices(t, [-0.14, -0.0725, 0.0], [0.14, 0.0725, 0.115]),
            )
        )
    elif task == "L2-03":
        a = spec.design.articles[0]
        samples = [
            (
                "left",
                (a.initial[0], a.initial[1] + 0.073 + step, a.initial[2]),
                (math.pi / 2, 0.0, -math.pi / 2),
                0.020,
            )
            for step in (0.0, 0.075, 0.15)
        ]
        for item in p["native_instances"][:2]:
            fixed.append(
                (
                    item["name"],
                    box_vertices(
                        pose_transform(item["pose"]),
                        [-0.07, -0.11, -0.01],
                        [0.07, 0.11, 0.01],
                    ),
                )
            )
        fixed.extend(
            (
                part.name,
                box_vertices(
                    origin_transform(part.position, part.rotation),
                    -np.asarray(part.size) / 2,
                    np.asarray(part.size) / 2,
                ),
            )
            for part in spec.design.fixtures
            if part.collision
        )
    elif task == "L2-01":
        a = spec.design.articles[0]
        x, y = a.initial[:2]
        # Complementary approaches avoid two downward wrists overlapping. The
        # horizontal cap grasp accounts for the pen's offset below the left TCP.
        cap_side = (-math.pi / 2, 0.0, -math.pi / 2)
        samples = [
            ("left", (x + 0.035, y, z + 0.024), down_left, 0.0162),
            ("left", (x + 0.035, y + 0.10, z + 0.140), down_left, 0.0162),
            ("right", (x + 0.135, y + 0.10, z + 0.1252), cap_side, 0.0174),
            ("right", (x + 0.18, y + 0.10, z + 0.1252), cap_side, 0.0174),
        ]
    elif task == "L2-04":
        a = spec.design.articles[0]
        for step in (0.0, a.goal[1] - a.initial[1]):
            for arm, side, rpy in (
                ("left", -1.0, down_left),
                ("right", 1.0, down_right),
            ):
                samples.append(
                    (
                        arm,
                        (a.initial[0] + side * 0.084, a.initial[1] + step, z + 0.038),
                        rpy,
                        0.045,
                    )
                )
    elif task == "L2-06":
        a = spec.design.articles[0]
        samples = [
            ("left", (p[0], p[1] - 0.007, z + 0.026), down_left, 0.023)
            for p in (a.initial, a.goal)
        ]
    else:
        raise ValueError(task)
    return samples, fixed


def check(task):
    _, config = resolve_task(task)
    spec, geometry = build_spec(task), Geometry(config)
    samples, fixtures = samples_and_fixtures(task, spec)
    seeds = {m.name: np.asarray(m.home_positions_rad) for m in config.robot.mounts}
    report = []
    left_boxes = None
    for arm, position, rpy, width in samples:
        obstacles = list(fixtures)
        if task in {"L2-01", "L2-04"} and arm == "right" and left_boxes is not None:
            obstacles += [("left/" + name, shape) for name, shape in left_boxes.items()]
        q, worst, margin = solve_clear(
            geometry,
            origin_transform(position, rpy),
            seeds[arm],
            width,
            obstacles,
            arm=arm,
            table_z=bench_geometry(config).top,
        )
        seeds[arm] = q
        if arm == "left":
            left_boxes, _ = geometry.boxes(arm, q, width)
        report.append(
            {
                "arm": arm,
                "tcp_position": list(position),
                "rpy": list(rpy),
                "width_m": width,
                "joints": q.tolist(),
                "worst_pair": worst[:2],
                "fixture_clearance_m": worst[2],
                "table_clearance_m": margin,
            }
        )
    return {"task": task, "passed": True, "samples": report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "tasks", nargs="*", default=["L2-07", "L2-03", "L2-01", "L2-04", "L2-06"]
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "reports/l2_asset_validation/clearance.json",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    results = []
    for task in args.tasks:
        try:
            result = check(task)
        except Exception as exc:  # noqa: BLE001 - persist diagnostics for every task
            result = {"task": task, "passed": False, "error": str(exc)}
        results.append(result)
        print(task, result["passed"], result.get("error", ""), flush=True)
        args.output.write_text(
            json.dumps(
                {
                    "scope": "Sampled IK and conservative CAD/table/fixture clearance, including selected bimanual holding poses. No continuous path, self-collision, physical grip or GPT success claim.",
                    "results": results,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
    return 0 if all(r["passed"] for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
