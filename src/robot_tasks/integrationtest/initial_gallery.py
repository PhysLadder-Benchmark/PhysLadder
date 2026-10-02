"""Capture current task setups without calling a model or manipulating props.

Each task runs in a fresh Isaac process through the existing scene check,
before robot preparation. No CUDA planner is loaded. The detail camera is for
documentation only and never enters model observations.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from ..paths import PROJECT_ROOT


def source_fingerprint():
    """Invalidate previews when task geometry, setup or camera configs change."""
    digest = hashlib.sha256()
    files = list((PROJECT_ROOT / "src/robot_tasks").rglob("*.py"))
    for folder in ("tasks", "robots", "cameras", "workflows"):
        files.extend((PROJECT_ROOT / "configs" / folder).glob("*.toml"))
    for path in sorted(files):
        digest.update(path.relative_to(PROJECT_ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def capture_detail(backend, task, output):
    import numpy as np
    from PIL import Image
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    prim = backend.world.stage.GetPrimAtPath(task.scene.root)
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    low, high = np.array(bounds.GetMin()), np.array(bounds.GetMax())
    if not np.isfinite([low, high]).all() or np.any(high < low):
        raise ValueError("Invalid task geometry bounds for preview camera")
    target = (low + high) / 2
    distance = max(.38, float(np.linalg.norm(high - low)) * 1.8)
    eye = target + np.array([.65, -.72, .90]) * distance
    rep = backend._rep
    camera = rep.create.camera(name="documentation_detail", position=tuple(eye),
                               look_at=tuple(target), clipping_range=(.005, 20))
    product = rep.create.render_product(camera, (1200, 900))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(product)
    try:
        # Render the physically reset initial state; do not advance physics.
        timestamp = backend.time_s()
        for _ in range(8):
            backend.world.render()
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=2)
            if not np.isclose(backend.time_s(), timestamp, rtol=0, atol=1e-9):
                raise RuntimeError("Preview unexpectedly advanced the simulation clock")
            data = np.asarray(rgb.get_data())
            if data.ndim == 3 and data.shape[:2] == (900, 1200) and data[..., :3].std() >= 1:
                break
        if data.ndim != 3 or data.shape[:2] != (900, 1200) or data[..., :3].std() < 1:
            raise RuntimeError("Documentation camera did not render a valid image")
        Image.fromarray(np.ascontiguousarray(data[..., :3], dtype=np.uint8)).save(output / "detail.png")
        return {"image": "detail.png", "camera_position": eye.tolist(),
                "camera_target": target.tolist(), "bounds": [low.tolist(), high.tolist()],
                "simulation_time_s": backend.time_s(), "model_input": False,
                "source_sha256": source_fingerprint()}
    finally:
        rgb.detach(product)
        product.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", nargs="?")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.all:
        if not args.task:
            parser.error("Provide a task ID or --all")
        from .scene_smoke import check

        return check(args.task, args.output, prepare=False, capture_hook=capture_detail)

    from ..registry import TASKS

    results = []
    for spec in TASKS.values():
        if not spec.task_id.startswith("L"):
            continue
        if not spec.implemented:
            results.append({"task": spec.task_id, "status": "real_only", "captured": False})
            continue
        folder = args.output / spec.task_id
        # Resume an interrupted export only when its scene capture passed.
        result_path = folder / "result.json"
        existing = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else {}
        if (existing.get("passed") and (folder / "detail.png").exists()
                and existing.get("preview", {}).get("source_sha256") == source_fingerprint()):
            result = existing
            code = 0
        else:
            with (args.output / (spec.task_id + ".log")).open("w", encoding="utf-8") as log:
                process = subprocess.run(
                    [sys.executable, "-m", __package__ + ".initial_gallery", spec.task_id,
                     "--output", str(folder)], stdout=log, stderr=subprocess.STDOUT,
                    cwd=PROJECT_ROOT, check=False,
                )
            code = process.returncode
            result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else {}
        entry = {"task": spec.task_id, "status": "captured" if result.get("passed") else "error",
                 "captured": bool(result.get("passed")), "returncode": code, "error": result.get("error")}
        results.append(entry)
        (args.output / "summary.json").write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    return 0 if all(r["status"] != "error" for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
