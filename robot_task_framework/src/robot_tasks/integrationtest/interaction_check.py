"""R5 physics-driven pick/place integration with independent contact/state evidence."""

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

import numpy as np

from ..backends.isaacsim import IsaacSimBackend
from ..backends.settings import SimulationSettings
from ..config import load_config
from ..recording import write_json
from ..robots.adapters import R5Adapter
from ..robots.audit import require_cartesian_sim_ready
from ..robots.transforms import pose_errors
from ..runtime import STORAGE_CONFIG
from ..types import Action, ArmCommand
from .interaction_metrics import lifted, placed
from .interaction_scene import InteractionScene, InteractionSettings
from .video import MotionVideo


def run_interaction_check(config_path: Path, output_root: Path, *, gui: bool = False,
                          run_directory: Path | None = None) -> tuple[Path, dict]:
    config = load_config(config_path)
    if config.arms != ("left", "right"):
        raise ValueError("The interaction check requires the R5 dual-arm platform")
    audit = require_cartesian_sim_ready(config.robot)
    options = InteractionSettings.from_config(config)
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    settings = SimulationSettings.from_config(config)
    if not math.isclose(config.control_dt_s * settings.fps, 1.0, abs_tol=1e-9):
        raise ValueError("Interaction check requires one video frame per control step")
    output = run_directory or output_root / ("R5_interaction_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("Interaction-check output must be a new directory")
    print(f"R5_CHECK_OUTPUT {output}", flush=True)
    for source, name in ((config.source, "config.toml"), (config.robot.source, "robot_profile.toml"),
                         (STORAGE_CONFIG, "storage_config.json"), (config.robot.asset_lock, "asset_source.lock.json")):
        shutil.copyfile(source, output / name)
    write_json(output / "asset_audit.json", audit.as_dict())
    write_json(output / "manifest.json", {
        "kind": "r5_pick_place_integration", "robot": "R5a", "benchmark_evaluated": False,
        "hardware_calibrated": False, "gpt_called": False, "seed": config.seed,
        "simulation": {**asdict(settings), "asset_directory": str(settings.asset_directory)},
        "interaction": asdict(options), "policy": "scripted known geometry, Cartesian waypoints and gripper widths",
        "source_sha256": {p.relative_to(Path(__file__).parents[1]).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    result = {"status": "starting", "passed": False, "task_evaluated": False, "task_success": None,
              "scope": "Scripted contact integration only; no benchmark/model success rate or hardware validation",
              "cases": [], "reset_checks": []}
    write_json(output / "result.json", result)
    backend, robot, video = IsaacSimBackend(), R5Adapter(), None
    start = time.monotonic()
    trace = (output / "interaction.jsonl").open("w", encoding="utf-8")
    case_name, active_arm = "setup", "left"
    downward = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
    try:
        backend.open(config)
        robot.initialize(backend, config)
        scene = InteractionScene(backend, config, options)
        video = MotionVideo(output, settings)

        def tick(phase, commands=()):
            if time.monotonic() - start > config.max_wall_time_s or video.count >= config.max_steps:
                raise TimeoutError("Interaction check exceeded its wall-time/step budget")
            action = Action(tuple(commands), config.control_dt_s)
            robot.apply(action)
            backend.step(config.control_dt_s)
            states = robot.state()
            state = states[active_arm]
            measured = scene.measure(active_arm)
            if not np.isfinite(measured["position_m"]).all() or measured["position_m"][2] < -0.02:
                raise RuntimeError("Object fell below the bench or state became nonfinite")
            row = {"case": case_name, "phase": phase, "time_s": backend.time_s(),
                   "action": asdict(action), "robot": {name: asdict(s) for name, s in states.items()},
                   "feedback": asdict(robot.feedback()), "object": measured,
                   "tcp_pose_world_wxyz": state.tcp_pose_world_wxyz, "gripper_width_m": state.gripper_width_m}
            trace.write(json.dumps(row, allow_nan=False) + "\n")
            video.add(backend.capture_rgb()[0])
            return row

        def hold(phase, seconds):
            return [tick(phase) for _ in range(math.ceil(seconds * settings.fps) + 1)]

        def move(phase, position, *, interpolate=True):
            initial = robot.state()[active_arm].tcp_pose_world_wxyz
            steps = max(1, math.ceil(math.dist(initial[:3], position) / options.waypoint_spacing_m)) if interpolate else 1
            samples = []
            for index in range(1, steps + 1):
                xyz = np.asarray(initial[:3]) + (np.asarray(position) - initial[:3]) * (index / steps)
                target = (*xyz, *downward)
                command = ArmCommand(active_arm, "cartesian_pose", tuple(float(v) for v in target))
                stable = 0
                for step in range(settings.fps * 10):
                    row = tick(phase, (command,) if step == 0 else ())
                    samples.append(row)
                    p_error, r_error = pose_errors(row["tcp_pose_world_wxyz"], target)
                    stable = stable + 1 if p_error < 0.005 and r_error < 0.02 else 0
                    if stable >= 2:
                        break
                else:
                    raise TimeoutError(f"{phase}: TCP did not reach waypoint {index}/{steps}: {p_error:.4f} m, {r_error:.4f} rad")
            print(f"R5_PHASE {case_name} {phase}", flush=True)
            trace.flush()
            return samples

        def gripper(phase, width):
            tick(phase, (ArmCommand(active_arm, "gripper", (width,)),))
            return hold(phase, 1.0)

        for case_name, active_arm, close_gripper in (
            ("left_pick_place", "left", True), ("right_pick_place", "right", True),
            ("open_gripper_negative", "left", False),
        ):
            scene.reset(active_arm)
            robot.reset(config.seed)
            # Fresh physics handles after reset; every episode starts from the same defined state.
            reset = scene.measure(active_arm)
            reset_error = math.dist(reset["position_m"], scene.source(active_arm))
            reset_joints = max(abs(q-h) for mount in config.robot.mounts
                               for q,h in zip(robot.state()[mount.name].joint_positions_rad, mount.home_positions_rad))
            result["reset_checks"].append({"case": case_name, "object_position_error_m": reset_error,
                                            "joint_error_rad": reset_joints,
                                            "passed": reset_error < 1e-5 and reset_joints < 1e-5})
            hold("settle", 0.75)
            write_json(output / "scene.json", backend.scene_report)
            write_json(output / "import.json", backend.import_report)
            if len(result["reset_checks"]) == 1:
                backend.save_scene(output / "scene.usda")
            source, destination = scene.source(active_arm), scene.destination(active_arm)
            initial_z = scene.measure(active_arm)["position_m"][2]
            gripper("open", options.open_width_m)
            move("approach", (*source[:2], options.approach_z_m), interpolate=False)
            move("descend", (*source[:2], options.grasp_tcp_z_m))
            gripper("close" if close_gripper else "skip_close", options.closed_width_m if close_gripper else options.open_width_m)
            move("lift", (*source[:2], options.lift_z_m))
            lift_samples = hold("lift_hold", options.stability_s)
            lift_check = lifted(lift_samples, initial_z, options)
            case = {"case": case_name, "arm": active_arm, "expected_success": close_gripper,
                    "source_m": source, "destination_m": destination, "lift": lift_check,
                    "interaction_success": False, "passed": False}
            result["cases"].append(case)
            write_json(output / "result.json", result)
            if close_gripper:
                if not lift_check["passed"]:
                    raise AssertionError(f"{case_name}: actual object lift/contact check failed: {lift_check}")
                transfer_samples = move("transfer", (*destination[:2], options.lift_z_m))
                carry = lifted(transfer_samples + hold("transfer_hold", options.stability_s), initial_z, options)
                case["carry"] = carry
                if not carry["passed"]:
                    raise AssertionError(f"{case_name}: object slipped during transfer")
                # Stop slightly above support to avoid commanding penetration under gravity droop.
                move("lower", (*destination[:2], options.grasp_tcp_z_m + 0.004))
                gripper("release", options.open_width_m)
                move("retreat", (*destination[:2], options.approach_z_m))
                hold("settle_after_release", 0.5)
                placement = placed(hold("placement_stability", options.stability_s), destination, options)
                case["placement"] = placement
                case["interaction_success"] = placement["passed"] and carry["passed"] and lift_check["passed"]
                case["passed"] = case["interaction_success"]
            else:
                # An open gripper must leave the cube on the bench, not report a grasp.
                displacement = max(math.dist(row["object"]["position_m"], source) for row in lift_samples)
                case["max_object_displacement_m"] = displacement
                case["passed"] = not lift_check["passed"] and displacement < 0.01
            write_json(output / "result.json", result)
        result.update(status="completed", passed=all(c["passed"] for c in result["cases"] + result["reset_checks"]),
                      frames=video.count)
    except (Exception, SystemExit, KeyboardInterrupt) as exc:
        result.update(status="error", passed=False, error=f"{type(exc).__name__}: {exc}", failed_case=case_name)
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
