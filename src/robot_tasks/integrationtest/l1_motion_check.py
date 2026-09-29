"""Replay the reported first-step failure with physical motion; no GPT score."""

from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import traceback
from uuid import uuid4

import numpy as np

from ..backends.isaacsim import IsaacSimBackend
from ..config import load_config, ExecutionSettings
from ..recording import finish_run, write_json
from ..robots.adapters import R5Adapter
from ..tasks.l1.task import L1Task
from ..types import Action, ArmCommand


def run_l1_motion_check(config_path, output_root, *, gui=False, run_directory=None):
    config = L1Task.prepare_config(load_config(config_path))
    execution = ExecutionSettings.from_config(config)
    if execution.motion_mode != "cartesian_linear_v1" or execution.gripper_timing != "open_before_motion":
        raise ValueError("l1-motion requires cartesian_linear_v1 and open_before_motion")
    if gui:
        config = replace(
            config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}}
        )
    output = run_directory or Path(output_root) / (
        "L1_motion_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8]
    )
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("Check output must be fresh")
    (output / "config.toml").write_bytes(config.source.read_bytes())
    result = {
        "passed": False,
        "task_evaluated": False,
        "success": None,
        "gpt_called": False,
        "scope": "Recorded pose replay, gripper sequence, known-bench rejection and recovery",
        "motions": [],
    }
    backend, robot, task = IsaacSimBackend(), R5Adapter(), L1Task()
    started = time.monotonic()
    try:
        backend.open(config)
        robot.initialize(backend, config)
        task.build_scene(backend, config)
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        from isaacsim.core.prims import RigidPrim
        from PIL import Image

        base = next(m.prim_path for m in config.robot.mounts if m.name == "left")
        contacts = RigidPrim(
            base + "/link[1-8]",
            name="motion_bench_contacts",
            reset_xform_properties=False,
            track_contact_forces=True,
            contact_filter_prim_paths_expr=["/World/Bench"],
            max_contact_count=4096,
            disable_stablization=False,
        )
        contacts.initialize()

        def advance():
            if time.monotonic() - started > config.max_wall_time_s:
                raise TimeoutError("Motion check exceeded wall-time budget")
            backend.step(config.control_dt_s)

        def frames(label):
            for frame in backend.capture_rgb():
                Image.frombytes("RGB", (frame.width, frame.height), frame.data).save(
                    output / f"{label}_{frame.camera}.png"
                )

        for _ in range(execution.settle_steps):
            advance()
        initial = robot.state()["left"]
        first_goal = (0.4, 0.06, 0.16, *initial.tcp_pose_world_wxyz[3:])
        frames("initial")
        with (output / "motion.jsonl").open("w", encoding="utf-8") as trace:

            def exercise(label, goal, width):
                robot.apply(
                    Action(
                        (ArmCommand("left", "cartesian_pose", goal), ArmCommand("left", "gripper", (width,))),
                        config.control_dt_s,
                    )
                )
                stable, peak_force, opening_drift = 0, 0.0, 0.0
                for tick in range(execution.max_action_steps):
                    advance()
                    state, feedback = robot.state()["left"], robot.feedback()
                    force = float(np.linalg.norm(contacts.get_contact_force_matrix(dt=config.physics_dt_s)))
                    peak_force = max(peak_force, force)
                    if (
                        label == "reported_first_step"
                        and feedback.metrics["arms"]["left"]["opening_before_motion"]
                    ):
                        opening_drift = max(
                            opening_drift,
                            max(
                                abs(a - b)
                                for a, b in zip(state.joint_positions_rad, initial.joint_positions_rad)
                            ),
                        )
                    trace.write(
                        json.dumps(
                            {
                                "label": label,
                                "tick": tick,
                                "requested_pose": goal,
                                "state": asdict(state),
                                "feedback": asdict(feedback),
                                "bench_force_n": force,
                            }
                        )
                        + "\n"
                    )
                    stable = stable + 1 if feedback.status == "completed" else 0
                    if stable >= execution.completion_samples:
                        break
                item = {
                    "label": label,
                    "completed": stable >= execution.completion_samples,
                    "duration_s": (tick + 1) * config.control_dt_s,
                    "peak_bench_force_n": peak_force,
                    "opening_joint_drift_rad": opening_drift,
                    "final_feedback": asdict(feedback),
                    "measured_gripper_width_m": state.gripper_width_m,
                }
                result["motions"].append(item)
                frames(label)
                if not item["completed"]:
                    robot.on_action_timeout()
                    raise RuntimeError(f"Motion did not complete: {label}")

            exercise("reported_first_step", first_goal, 0.07)
            exercise("reported_second_step", (0.4, 0.1, 0.2, 1, 0, 0, 0), 0.07)
            before = dict(robot.targets)
            try:
                robot.apply(
                    Action(
                        (
                            ArmCommand(
                                "left",
                                "cartesian_pose",
                                (0.4, 0.42, 0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0),
                            ),
                            ArmCommand("left", "gripper", (0.012,)),
                        ),
                        config.control_dt_s,
                    )
                )
            except ValueError as exc:
                result["unsafe_rejection"] = str(exc)
                result["rejection_atomic"] = before == robot.targets
            else:
                raise RuntimeError("Unsafe through-table command was not rejected")
            exercise(
                "downward_observation_pose",
                (0.36, 0.42, 0.25, math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0),
                0.07,
            )
            # A combined closing command must complete arm motion before closing.
            exercise("close_after_motion", (0.36, 0.42, 0.22, math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0), 0.02)
        result["passed"] = bool(
            result.get("rejection_atomic")
            and "table_collision" in result.get("unsafe_rejection", "")
            and all(m["completed"] and m["peak_bench_force_n"] < 1.0 for m in result["motions"])
            and result["motions"][0]["opening_joint_drift_rad"] < 0.02
            and abs(result["motions"][0]["measured_gripper_width_m"] - 0.07) < 0.003
            and abs(result["motions"][-1]["measured_gripper_width_m"] - 0.02) < 0.003
        )
        result["status"] = "completed"
        write_json(output / "scene.json", backend.scene_report)
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="error", error=f"{type(exc).__name__}: {exc}")
        (output / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
    finally:
        result["wall_time_s"] = time.monotonic() - started
        finish_run(output, result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        backend.close()
    return output, result
