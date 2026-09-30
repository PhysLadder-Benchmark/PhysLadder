"""Manual Isaac scene/startup check. Never loads an API key or calls GPT.

Run in the Isaac environment: python -m robot_tasks.integrationtest.scene_smoke --all
Each scene uses a fresh process; videos/model success rates are separate experiments.
"""

import argparse
import json
import os
import subprocess
import sys
import traceback
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

from ..paths import PROJECT_ROOT


def check(task_id, output, *, prepare=True, prop_probe=False):
    import numpy as np
    from PIL import Image

    from ..backends.isaacsim import IsaacSimBackend
    from ..registry import resolve_task
    from ..robots.adapters import R5Adapter
    from ..robots.preparation import preparation_action
    from ..types import EvaluationState

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    output.mkdir(parents=True, exist_ok=True)
    spec, config = resolve_task(task_id)
    config = spec.factory.prepare_config(config)
    backend, robot, task = IsaacSimBackend(), R5Adapter(), spec.factory()
    backend.run_directory = output
    result = {
        "task": task_id,
        "passed": False,
        "gpt_called": False,
        "scope": "scene, physical reset, RGB-D and initial scoring; not task-solving evidence",
        "preparation_requested": prepare,
    }
    try:
        backend.open(config)
        if prepare:
            robot.initialize(backend, config)
        else:
            # Physical reset/camera checks do not need a CUDA planning worker.
            from ..config import ExecutionSettings
            from ..robots.kinematics import R5Kinematics

            robot.config, robot.backend = config, backend
            robot.execution = ExecutionSettings.from_config(config)
            robot.kinematics = R5Kinematics(config.robot)
            robot.instances = backend.add_r5(config.robot)
        task.build_scene(backend, config)
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        for _ in range(24):
            backend.step(config.control_dt_s)
        if prepare:
            robot.apply(
                getattr(spec.factory, "preparation_action", preparation_action)(config)
            )
            completed = 0
            for _ in range(120):
                backend.step(config.control_dt_s)
                completed = (
                    completed + 1 if robot.feedback().status == "completed" else 0
                )
                if completed >= 3:
                    break
            if completed < 3:
                raise RuntimeError(
                    "Startup preparation failed: " + str(asdict(robot.feedback()))
                )
        rows = task.scene.measure()
        frames = backend.capture_rgb()
        result["cameras"] = []
        for frame in frames:
            from ..cameras.depth import depth_array

            depth = depth_array(frame)
            if not np.any(np.isfinite(depth) & (depth > 0)):
                raise RuntimeError("Empty depth: " + frame.camera)
            Image.frombytes("RGB", (frame.width, frame.height), frame.data).save(
                output / (frame.camera + ".png")
            )
            if np.frombuffer(frame.data, dtype=np.uint8).std() < 1:
                raise RuntimeError("Blank camera: " + frame.camera)
            result["cameras"].append(frame.camera)
        # Initial state must be running, never already successful or physically failed.
        assessments = []
        for _ in range(14):
            backend.step(config.control_dt_s)
            assessment = task.evaluate(
                EvaluationState(
                    {"simulation_time_s": backend.time_s(), "robot": robot.state()}
                ),
                robot.feedback(),
            )
            assessments.append(assessment.status)
        if any(s != "running" for s in assessments):
            raise RuntimeError(
                "Invalid initial task assessment: " + str(asdict(assessment))
            )
        result.update(
            passed=True,
            initial_assessment=asdict(assessment),
            objects_before=rows,
            scene=backend.scene_report,
            prepared=prepare,
        )
        if prop_probe:
            from .prop_physics import run

            result["prop_probe"] = run(task.scene)
            for frame in backend.capture_rgb():
                Image.frombytes("RGB", (frame.width, frame.height), frame.data).save(
                    output / ("prop_probe_" + frame.camera + ".png")
                )
    except Exception as exc:  # noqa: BLE001 - a diagnostic must persist every worker failure
        traceback.print_exc()
        result["passed"] = False
        result["error"] = str(exc)
    finally:
        try:
            (output / "result.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False),
                encoding="utf-8",
            )
            print(
                "SCENE_SMOKE",
                task_id,
                "PASS" if result["passed"] else "FAIL",
                flush=True,
            )
        finally:
            try:
                robot.close()
            finally:
                backend.close()
    return 0 if result["passed"] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", nargs="?")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--no-preparation", action="store_true")
    parser.add_argument("--prop-probe", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.output or PROJECT_ROOT / "runs" / (
        "scene_smoke_" + datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    )
    root.mkdir(parents=True, exist_ok=True)
    if args.all:
        from ..registry import TASKS

        results = []
        for spec in TASKS.values():
            if not spec.task_id.startswith("L") or not spec.implemented:
                continue
            command = [
                sys.executable,
                "-m",
                __package__ + ".scene_smoke",
                spec.task_id,
                "--output",
                str(root / spec.task_id),
            ]
            if args.no_preparation:
                command.append("--no-preparation")
            if args.prop_probe:
                command.append("--prop-probe")
            with (root / (spec.task_id + ".log")).open("w", encoding="utf-8") as log:
                process = subprocess.run(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    cwd=PROJECT_ROOT,
                    check=False,
                )
            result_path = root / spec.task_id / "result.json"
            result = (
                json.loads(result_path.read_text(encoding="utf-8"))
                if result_path.exists()
                else {"error": "worker did not produce result"}
            )
            entry = {
                "task": spec.task_id,
                "passed": result.get("passed", False),
                "returncode": process.returncode,
                "error": result.get("error"),
            }
            results.append(entry)
            (root / "summary.json").write_text(
                json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            print(json.dumps(entry, ensure_ascii=False), flush=True)
        return 0 if all(r["passed"] and r["returncode"] == 0 for r in results) else 2
    if not args.task:
        parser.error("Provide a task ID or --all")
    return check(
        args.task, root, prepare=not args.no_preparation, prop_probe=args.prop_probe
    )


if __name__ == "__main__":
    raise SystemExit(main())
