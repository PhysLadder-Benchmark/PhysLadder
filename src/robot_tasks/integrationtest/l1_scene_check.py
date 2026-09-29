"""No-model scene check. Fixture pose injection is NEVER a benchmark success."""

from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
import json
import traceback
import time
import numpy as np

from ..backends.isaacsim import IsaacSimBackend
from ..config import load_config
from ..recording import finish_run, write_json
from ..robots.adapters import R5Adapter
from ..tasks.l1.task import L1Task
from ..types import EvaluationState


def run_l1_scene_check(config_path, output_root, *, gui=False, run_directory=None):
    config = L1Task.prepare_config(load_config(config_path))
    if gui:
        config = replace(
            config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}}
        )
    output = run_directory or Path(output_root) / (
        config.task_id + "_scene_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8]
    )
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("Check output must be fresh")
    (output / "config.toml").write_bytes(config.source.read_bytes())
    backend, robot, task = IsaacSimBackend(), R5Adapter(), L1Task()
    result = {
        "status": "starting",
        "passed": False,
        "task_evaluated": False,
        "success": None,
        "scope": "Engineering reset/RGB/contact/fixture check with pose injection; no GPT ability score",
        "gpt_called": False,
        "task_id": config.task_id,
    }
    started = time.monotonic()
    try:
        backend.open(config)
        robot.initialize(backend, config)
        task.build_scene(backend, config)
        support_probe = None
        if config.task_id == "L1-01":
            # Goal injection alone can pass with a missing rack: the tube can
            # stand on the bench. A separate falling cube must rest on the rack.
            # This object exists only in this engineering check, never a task.
            from isaacsim.core.api.objects import DynamicCuboid
            from pxr import PhysxSchema

            rack = task.scene.design
            support_probe = backend.world.scene.add(DynamicCuboid(
                "/World/RackSupportProbe", name="rack_support_probe", size=.006, mass=.01,
                position=np.array([rack.fixture_pose[0],
                                   rack.fixture_pose[1] + .054, rack.fixture_top + .025]),
                color=np.array([.9, .1, .1]),
            ))
            collision = PhysxSchema.PhysxCollisionAPI.Apply(support_probe.prim)
            collision.CreateContactOffsetAttr(.001)
            collision.CreateRestOffsetAttr(0.)
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)

        def settle(seconds):
            assessment = None
            for _ in range(round(seconds / config.control_dt_s)):
                if time.monotonic() - started > config.max_wall_time_s:
                    raise TimeoutError("L1 scene check timed out")
                backend.step(config.control_dt_s)
                assessment = task.evaluate(
                    EvaluationState({"simulation_time_s": backend.time_s(), "robot": robot.state()}),
                    robot.feedback(),
                )
            return assessment

        def images(label):
            from PIL import Image, ImageDraw

            frames = backend.capture_rgb()
            if tuple(f.camera for f in frames) != config.cameras:
                raise RuntimeError("Missing camera")
            preview = Image.new("RGB", (640 * 3, 512))
            for i, f in enumerate(frames):
                img = Image.frombytes("RGB", (f.width, f.height), f.data)
                img.save(output / (label + "_" + f.camera + ".png"))
                preview.paste(img.resize((640, 480)), (i * 640, 32))
                ImageDraw.Draw(preview).text((i * 640 + 10, 8), f.camera, fill="white")
            preview.save(output / (label + "_views.png"))
            return {f.camera: float(np.frombuffer(f.data, np.uint8).std()) for f in frames}

        initial = settle(2.0)
        rgb = images("initial")
        result["initial"] = initial.metrics
        result["initial_not_success"] = initial.status == "running" and not initial.metrics["goal_complete"]
        result["rgb_std"] = rgb
        if support_probe is not None:
            height = float(support_probe.get_world_pose()[0][2])
            expected = task.scene.design.fixture_top + .003
            result["rack_support_probe"] = {
                "height_m": height, "expected_height_m": expected,
                "passed": bool(abs(height - expected) < .002
                               and np.linalg.norm(support_probe.get_linear_velocity()) < .01),
            }
        # Physics-only fixture validation; deliberately bypass grasp history.
        # Benchmark code has no such setter in its action path.
        for a in task.scene.design.articles:
            body = task.scene.bodies[a.name]
            body.set_world_pose(np.array(a.goal[:3]), np.array(a.goal[3:]))
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
        target = settle(6.0)
        images("fixture")
        result["fixture_probe"] = target.metrics
        result["pose_injection_not_success"] = target.status != "succeeded"
        result["fixture_stable"] = all(row["stable"] for row in target.metrics["criteria"].values())
        # Slot crossing requires history; validate containment separately here.
        if task.scene.design.kind == "slot":
            xyz = np.asarray(next(iter(target.metrics["objects"].values()))["corners"])
            goal = task.scene.design.articles[0].goal
            seated = bool(
                xyz[:, 2].max() < 0.096
                and xyz[:, 2].min() >= 0.003
                and np.max(abs(xyz[:, :2] - goal[:2])) < 0.060
            )
        else:
            seated = target.metrics["goal_complete"]
        result["fixture_goal_geometry"] = seated
        write_json(output / "scene.json", backend.scene_report)
        backend.save_scene(output / "scene.usda")
        # Same seed resets the same layout and clears scorer state.
        original = task.scene.initial.copy()
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        result["reset_reproducible"] = original == task.scene.initial and not task.scorer.grasped
        result.update(
            status="completed",
            passed=bool(
                result["initial_not_success"]
                and result["pose_injection_not_success"]
                and result["fixture_stable"]
                and seated
                and result["reset_reproducible"]
                and min(rgb.values()) > 3
                and result.get("rack_support_probe", {}).get("passed", True)
            ),
        )
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="error", error=f"{type(exc).__name__}: {exc}")
        (output / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
    finally:
        result["wall_time_s"] = time.monotonic() - started
        finish_run(output, result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        try:
            if gui and result["status"] == "completed":
                from .scene_preview import hold_scene_preview
                hold_scene_preview(backend)
        finally:
            backend.close()
    return output, result
