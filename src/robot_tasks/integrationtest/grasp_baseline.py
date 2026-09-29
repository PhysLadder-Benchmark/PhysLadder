"""Known-geometry engineering baseline for the L1 executor; never a GPT score."""

from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import traceback
from uuid import uuid4

from ..backends.isaacsim import IsaacSimBackend
from ..backends.settings import SimulationSettings
from ..config import ExecutionSettings, load_config
from ..recording import finish_run, write_json
from ..robots.adapters import R5Adapter
from ..robots.preparation import preparation_action
from ..tasks.pick_place_scene import InteractionScene, InteractionSettings
from ..tasks.pick_place_metrics import lifted, placed
from ..types import Action, ArmCommand
from ..video import MotionVideo


def run_grasp_baseline(config_path, output_root, *, gui=False, run_directory=None):
    config = load_config(config_path)
    execution = ExecutionSettings.from_config(config)
    if config.arms != ("left",) or execution.gripper_completion != "stall_aware_v1":
        raise ValueError("l1-grasp requires left arm and stall_aware_v1")
    if execution.motion_mode != "cartesian_linear_v1" or execution.startup_pose != "downward_ready_v1":
        raise ValueError("l1-grasp requires the L1 Cartesian executor and downward preparation")
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    settings, options = SimulationSettings.from_config(config), InteractionSettings.from_config(config)
    output = run_directory or Path(output_root) / (
        "L1_grasp_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("Check output must be fresh")
    for source, name in ((config.source, "config.toml"), (config.robot.source, "robot_profile.toml"),
                         (config.robot.asset_lock, "asset_source.lock.json"),
                         (config.source.parent / settings.camera_profile, "camera_profile.toml")):
        (output / name).write_bytes(source.read_bytes())
    write_json(output / "manifest.json", {
        "kind": "l1_executor_grasp_baseline_v1", "task_evaluated": False, "success": None,
        "gpt_called": False, "object_pose_writes": "reset only", "attachments": False,
        "ground_truth_use": "Known geometry for engineering script; measured truth for assertions only",
        "scope": "40 mm cube; not an L1 task completion or an eight-task reachability certificate",
        "execution": asdict(execution), "seed": config.seed,
        "source_sha256": {str(p.relative_to(Path(__file__).parents[1])): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    result = {"passed": False, "task_evaluated": False, "success": None, "gpt_called": False, "cases": []}
    backend, robot, videos = IsaacSimBackend(), R5Adapter(), {}
    started, ticks = time.monotonic(), 0
    case = None
    trace = (output / "grasp.jsonl").open("w", encoding="utf-8")
    try:
        backend.open(config)
        robot.initialize(backend, config)
        scene = InteractionScene(backend, config, options)
        write_json(output / "scene.json", backend.scene_report)
        downward = (math.sqrt(.5), 0., math.sqrt(.5), 0.)
        inactive_initial = None

        def tick(phase):
            nonlocal ticks
            if time.monotonic() - started > config.max_wall_time_s or ticks >= config.max_steps:
                raise TimeoutError("Engineering baseline exceeded budget")
            backend.step(config.control_dt_s)
            ticks += 1
            states, feedback = robot.state(), robot.feedback()
            state = states["left"]
            row = {"case": case["name"], "phase": phase, "time_s": backend.time_s(),
                   "robot": {name: asdict(s) for name, s in states.items()},
                   "object": scene.measure("left"), "feedback": asdict(feedback),
                   "tcp_pose_world_wxyz": state.tcp_pose_world_wxyz, "gripper_width_m": state.gripper_width_m}
            drift = max(abs(a-b) for a,b in zip(states["right"].joint_positions_rad, inactive_initial))
            case["inactive_max_drift_rad"] = max(case["inactive_max_drift_rad"], drift)
            if drift > .01:
                raise AssertionError("Inactive arm moved")
            trace.write(json.dumps(row, allow_nan=False) + "\n")
            for frame in backend.capture_rgb():
                if frame.camera not in videos:
                    folder = output / "video" / frame.camera
                    folder.mkdir(parents=True)
                    videos[frame.camera] = MotionVideo(folder, replace(settings, width=frame.width, height=frame.height))
                videos[frame.camera].add(frame)
            return row

        def hold(phase, seconds=1.):
            return [tick(phase) for _ in range(math.ceil(seconds/config.control_dt_s)+1)]

        def execute(phase, action):
            trace.write(json.dumps({"case": case["name"], "phase": phase, "action": asdict(action)}) + "\n")
            robot.apply(action)
            stable = 0
            for _ in range(execution.max_action_steps):
                row = tick(phase)
                status = row["feedback"]["status"]
                if status in {"blocked", "error"}:
                    raise AssertionError(f"{phase}: {row['feedback']}")
                stable = stable + 1 if status == "completed" else 0
                if stable >= execution.completion_samples:
                    case["motions"].append({"phase": phase, "feedback": row["feedback"]})
                    print(f"R5_PHASE {case['name']} {phase}", flush=True)
                    trace.flush()
                    return row
            robot.on_action_timeout()
            raise AssertionError(f"{phase}: action timed out: {row['feedback']}")

        def move(phase, xyz, width=None):
            commands = [ArmCommand("left", "cartesian_pose", (*xyz, *downward))]
            if width is not None:
                commands.append(ArmCommand("left", "gripper", (width,)))
            return execute(phase, Action(tuple(commands), config.control_dt_s))

        def grip(phase, width):
            return execute(phase, Action((ArmCommand("left", "gripper", (width,)),), config.control_dt_s))

        # Same reset, two positive repetitions and an open-gripper negative control.
        for name, close in (("pick_place_1", True), ("pick_place_2", True), ("open_gripper_negative", False)):
            case = {"name": name, "passed": False, "motions": [], "inactive_max_drift_rad": 0.}
            result["cases"].append(case)
            scene.reset("left")
            robot.reset(config.seed)
            inactive_initial = robot.state()["right"].joint_positions_rad
            hold("settle")
            execute("preparation", preparation_action(config))
            source, destination = scene.source("left"), scene.destination("left")
            initial_z = scene.measure("left")["position_m"][2]
            grip("open", options.open_width_m)
            move("approach", (*source[:2], options.approach_z_m))
            move("descend", (*source[:2], options.grasp_tcp_z_m))
            grip("close" if close else "skip_close", options.closed_width_m if close else options.open_width_m)
            move("lift", (*source[:2], options.lift_z_m), options.closed_width_m if close else options.open_width_m)
            samples = hold("lift_hold", options.stability_s)
            case["lift"] = lifted(samples, initial_z, options)
            if close:
                if not case["lift"]["passed"]:
                    raise AssertionError(f"Physical lift failed: {case['lift']}")
                move("transfer", (*destination[:2], options.lift_z_m), options.closed_width_m)
                case["carry"] = lifted(hold("carry_hold", options.stability_s), initial_z, options)
                if not case["carry"]["passed"]:
                    raise AssertionError("Cube slipped during transport")
                move("lower", (*destination[:2], options.grasp_tcp_z_m + .004))
                grip("release", options.open_width_m)
                move("retreat", (*destination[:2], options.approach_z_m))
                hold("settle_after_release", .5)
                case["placement"] = placed(hold("placement_hold", options.stability_s), destination, options)
                case["passed"] = case["placement"]["passed"]
            else:
                case["maximum_displacement_m"] = max(math.dist(r["object"]["position_m"], source) for r in samples)
                case["passed"] = not case["lift"]["passed"] and case["maximum_displacement_m"] < .01
            write_json(output / "result.json", result)
            if not case["passed"]:
                raise AssertionError(f"Case failed: {name}")
        result.update(passed=True, status="completed")
        write_json(output / "scene.json", backend.scene_report)
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="error", error=f"{type(exc).__name__}: {exc}")
        (output / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
    finally:
        trace.close()
        for camera, video in videos.items():
            try:
                video.close()
            except Exception as exc:
                result.setdefault("recording_errors", {})[camera] = f"{type(exc).__name__}: {exc}"
                result.update(passed=False, status="error")
        result.update(wall_time_s=time.monotonic()-started, control_ticks=ticks)
        finish_run(output, result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        backend.close()
    return output, result
