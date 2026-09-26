"""Measured R5 integration check, separate from manipulation-task evaluation."""

from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import traceback
from uuid import uuid4

from ..backends.isaacsim import IsaacSimBackend
from ..backends.settings import SimulationSettings
from ..config import load_config
from ..recording import write_json
from ..runtime import STORAGE_CONFIG
from ..robots.adapters import R5Adapter
from ..robots.audit import require_joint_sim_ready
from ..types import Action, ArmCommand
from .video import MotionVideo


def run_motion_check(config_path: Path, output_root: Path, *, gui: bool = False,
                     run_directory: Path | None = None) -> tuple[Path, dict]:
    config = load_config(config_path)
    if config.arms != ("left", "right"):
        raise ValueError("The R5 integration check requires both active arms")
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw.get("simulation", {}), "headless": False}})
    audit = require_joint_sim_ready(config.robot)
    settings = SimulationSettings.from_config(config)
    if not math.isclose(config.control_dt_s * settings.fps, 1.0, abs_tol=1e-9):
        raise ValueError("Motion-check control_dt_s must equal 1 / simulation.fps")
    output = run_directory or output_root / ("R5_motion_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(path.name != "process.log" for path in output.iterdir()):
        raise ValueError("Motion-check output must be a new directory")
    print(f"R5_CHECK_OUTPUT {output}", flush=True)
    for source, name in ((config.source, "config.toml"), (config.robot.source, "robot_profile.toml"),
                          (STORAGE_CONFIG, "storage_config.json"),
                          (config.robot.asset_lock, "asset_source.lock.json")):
        shutil.copyfile(source, output / name)
    write_json(output / "asset_audit.json", audit.as_dict())
    write_json(output / "manifest.json", {
        "kind": "r5_joint_motion_integration", "robot": "R5a", "task_evaluated": False,
        "hardware_calibrated": False, "gpt_called": False, "seed": config.seed,
        "simulation": {**asdict(settings), "asset_directory": str(settings.asset_directory)},
        "source_sha256": {p.relative_to(Path(__file__).parents[1]).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    result = {"passed": False, "status": "starting", "task_evaluated": False, "task_success": None,
              "scope": "R5 dual-arm joint/gripper drives, state, reset and overview RGB; no IK, grasping or GPT",
              "joint_checks": [], "gripper_checks": []}
    write_json(output / "result.json", result)
    backend, robot, video = IsaacSimBackend(), R5Adapter(), None
    start = time.monotonic()
    trace = (output / "motion.jsonl").open("w", encoding="utf-8")
    try:
        backend.open(config)
        robot.initialize(backend, config)
        write_json(output / "scene.json", backend.scene_report)
        write_json(output / "import.json", backend.import_report)
        backend.reset(config.seed)
        robot.reset(config.seed)
        for _ in range(3):
            backend.capture_rgb()
        video = MotionVideo(output, settings)
        home = {name: target[:6] for name, target in robot.initial_targets.items()}
        midpoint = sum(config.robot.gripper.width_limits_m) / 2
        base_errors = []
        velocity_ratios = []
        limits = {limit.joint: limit for limit in config.robot.operating_limits}

        def advance(label, commands):
            if time.monotonic() - start > config.max_wall_time_s:
                raise TimeoutError("R5 integration check exceeded its wall-clock budget")
            action = Action(tuple(commands), config.control_dt_s)
            robot.apply(action)
            backend.step(config.control_dt_s)
            states = robot.state()
            bases = {name: [float(v) for v in arm.get_world_pose()[0]] for name, arm in robot.instances.items()}
            base_errors.append(max(abs(v - mount.base_pose_wxyz[i]) for mount in config.robot.mounts
                                   for i, v in enumerate(bases[mount.name])))
            velocity_ratios.append(max(abs(value) / limits[joint].velocity for state in states.values()
                for joint, value in zip(config.robot.joint_names + config.robot.gripper.joint_names,
                                        state.joint_velocities_rad_s + state.finger_velocities_m_s)))
            trace.write(json.dumps({"phase": label, "time_s": backend.time_s(),
                                    "targets": robot.targets, "state": {k: asdict(v) for k, v in states.items()},
                                    "base_positions_m": bases}, allow_nan=False) + "\n")
            video.add(backend.capture_rgb()[0])
            return states, bases

        def hold(label, seconds=0.25):
            for _ in range(round(seconds * settings.fps)):
                advance(label, ())

        hold("settle", 1.0)
        initial_bases = {name: tuple(float(v) for v in arm.get_world_pose()[0]) for name, arm in robot.instances.items()}
        write_json(output / "scene.json", backend.scene_report)
        write_json(output / "import.json", backend.import_report)
        backend.save_scene(output / "scene.usda")
        for name in config.arms:
            other = "right" if name == "left" else "left"
            for index, joint in enumerate(config.robot.joint_names):
                label = f"{name}_{joint}"
                print(f"R5_PHASE {label}", flush=True)
                measured, tracking, drift = [], [], []
                for step in range(1, settings.fps * 2 + 1):
                    target = list(home[name])
                    target[index] += 0.16 * math.sin(math.pi * step / (settings.fps * 2))
                    states, _ = advance(label, [ArmCommand(name, "joint_position", tuple(target))])
                    measured.append(states[name].joint_positions_rad[index])
                    tracking.append(abs(states[name].joint_positions_rad[index] - target[index]))
                    drift.append(max(abs(q - h) for q, h in zip(states[other].joint_positions_rad, home[other])))
                response = max(measured) - home[name][index]
                check = {"arm": name, "joint": joint, "positive_response_rad": response,
                         "max_tracking_error_rad": max(tracking), "other_arm_max_error_rad": max(drift),
                         "passed": response > 0.10 and max(tracking) < 0.04 and max(drift) < 0.02}
                result["joint_checks"].append(check)
                hold(label + "_hold")
                trace.flush()
                write_json(output / "result.json", result)

        for name in config.arms:
            label = f"{name}_gripper"
            print(f"R5_PHASE {label}", flush=True)
            widths, tracking = [], []
            for step in range(1, settings.fps * 3 + 1):
                width = midpoint + 0.026 * math.sin(2 * math.pi * step / (settings.fps * 3))
                states, _ = advance(label, [ArmCommand(name, "gripper", (width,))])
                widths.append(states[name].gripper_width_m)
                tracking.append(abs(widths[-1] - width))
            span = max(widths) - min(widths)
            result["gripper_checks"].append({"arm": name, "width_range_m": span,
                "max_tracking_error_m": max(tracking), "passed": span > 0.04 and max(tracking) < 0.004})
            hold(label + "_hold")

        print("R5_PHASE simultaneous", flush=True)
        both_errors = []
        for step in range(1, settings.fps * 2 + 1):
            commands = []
            for name, sign in (("left", 1), ("right", -1)):
                target = list(home[name])
                target[0] += sign * 0.16 * math.sin(math.pi * step / (settings.fps * 2))
                commands.append(ArmCommand(name, "joint_position", tuple(target)))
            states, bases = advance("simultaneous", commands)
            both_errors.append(max(abs(states[c.arm].joint_positions_rad[0] - c.values[0]) for c in commands))
        result["simultaneous_max_error_rad"] = max(both_errors)
        result["base_max_drift_m"] = max(abs(v - initial_bases[name][i]) for name, values in bases.items() for i, v in enumerate(values))
        result["base_placement_max_error_m"] = max(abs(v - mount.base_pose_wxyz[i])
            for mount in config.robot.mounts for i, v in enumerate(bases[mount.name]))
        # Displace with drives, then exercise both simulator and robot reset.
        displaced = [ArmCommand(name, "joint_position", (home[name][0] + 0.12,) + home[name][1:]) for name in config.arms]
        advance("before_reset", displaced)
        hold("before_reset_hold", 0.5)
        backend.reset(config.seed)
        robot.reset(config.seed)
        reset_states = robot.state()
        result["reset_max_error_rad"] = max(abs(q - h) for name, state in reset_states.items()
                                             for q, h in zip(state.joint_positions_rad, home[name]))
        hold("after_reset", 1.0)
        result["base_placement_max_error_m"] = max(base_errors)
        result["sampled_max_velocity_limit_ratio"] = max(velocity_ratios)
        result.update({"status": "completed", "frames": video.count,
            "maximum_pixel_change": video.maximum_pixel_change,
            "passed": all(c["passed"] for c in result["joint_checks"] + result["gripper_checks"])
                and max(both_errors) < 0.04 and result["base_max_drift_m"] < 0.001
                and result["base_placement_max_error_m"] < 0.001 and result["reset_max_error_rad"] < 1e-5
                and result["sampled_max_velocity_limit_ratio"] < 1.01
                and video.count > 20 and video.maximum_pixel_change > 0.05})
    except (Exception, SystemExit, KeyboardInterrupt) as exc:
        result.update(status="error", passed=False, error=f"{type(exc).__name__}: {exc}")
        (output / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        print(f"R5_CHECK_ERROR {exc}", flush=True)
    finally:
        trace.close()
        try:
            if video is not None:
                video.close()
        except Exception as exc:
            result.update(status="error", passed=False, video_error=str(exc))
        result["wall_time_s"] = time.monotonic() - start
        write_json(output / "result.json", result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        backend.close()
    return output, result
