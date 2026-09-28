"""Free-space R5 TCP integration check; not a manipulation benchmark."""

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
from ..video import MotionVideo
from ..recording import write_json
from ..runtime import STORAGE_CONFIG
from ..robots.adapters import R5Adapter
from ..robots.audit import require_cartesian_sim_ready
from ..robots.kinematics import IKError
from ..robots.transforms import pose_errors
from ..types import Action, ArmCommand


def run_cartesian_check(config_path: Path, output_root: Path, *, gui: bool = False,
                        run_directory: Path | None = None) -> tuple[Path, dict]:
    config = load_config(config_path)
    if config.arms != ("left", "right"):
        raise ValueError("Cartesian integration check requires both R5 arms")
    audit = require_cartesian_sim_ready(config.robot)
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    settings = SimulationSettings.from_config(config)
    if not math.isclose(config.control_dt_s * settings.fps, 1.0, abs_tol=1e-9):
        raise ValueError("Cartesian-check control_dt_s must equal 1 / simulation.fps")
    output = run_directory or output_root / ("R5_cartesian_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(path.name != "process.log" for path in output.iterdir()):
        raise ValueError("Cartesian-check output must be a new directory")
    print(f"R5_CHECK_OUTPUT {output}", flush=True)
    for source, name in ((config.source, "config.toml"), (config.robot.source, "robot_profile.toml"),
                         (STORAGE_CONFIG, "storage_config.json"),
                         (config.robot.asset_lock, "asset_source.lock.json")):
        shutil.copyfile(source, output / name)
    write_json(output / "asset_audit.json", audit.as_dict())
    write_json(output / "manifest.json", {
        "kind": "r5_cartesian_integration", "robot": "R5a", "task_evaluated": False,
        "hardware_calibrated": False, "gpt_called": False,
        "tcp_source": config.robot.tcp_source, "tcp_pose_wxyz": config.robot.tcp_pose_wxyz,
        "simulation": {**asdict(settings), "asset_directory": str(settings.asset_directory)},
        "source_sha256": {p.relative_to(Path(__file__).parents[1]).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    result = {"status": "starting", "passed": False, "task_evaluated": False, "task_success": None,
              "scope": "Free-space pose goals via IK and physics drives; no grasping, collision planning, force control or GPT",
              "pose_checks": [], "rejection_checks": [],
              "tolerances": {"position_m": 0.005, "orientation_rad": 0.02,
                             "fk_vs_physx_position_m": 0.0005, "fk_vs_physx_orientation_rad": 0.002}}
    write_json(output / "result.json", result)
    backend, robot, video = IsaacSimBackend(), R5Adapter(), None
    start = time.monotonic()
    trace = (output / "motion.jsonl").open("w", encoding="utf-8")
    fk_errors = []
    try:
        backend.open(config)
        robot.initialize(backend, config)
        backend.reset(config.seed)
        robot.reset(config.seed)
        for _ in range(3):
            backend.capture_rgb()
        video = MotionVideo(output, settings)
        write_json(output / "scene.json", backend.scene_report)
        write_json(output / "import.json", backend.import_report)
        backend.save_scene(output / "scene.usda")

        def advance(label, commands=()):
            if time.monotonic() - start > config.max_wall_time_s:
                raise TimeoutError("R5 Cartesian check exceeded its wall-clock budget")
            if video.count >= config.max_steps:
                raise TimeoutError("R5 Cartesian check exceeded its step budget")
            action = Action(tuple(commands), config.control_dt_s)
            robot.apply(action)
            backend.step(config.control_dt_s)
            states = robot.state()
            feedback = robot.feedback()
            for name, state in states.items():
                predicted = robot.kinematics.forward(state.joint_positions_rad, name)
                fk_errors.append(pose_errors(state.tcp_pose_world_wxyz, predicted))
            trace.write(json.dumps({"phase": label, "time_s": backend.time_s(), "action": asdict(action),
                                    "targets": robot.targets, "tcp_targets_world_wxyz": robot.cartesian_targets,
                                    "state": {k: asdict(v) for k, v in states.items()}, "feedback": asdict(feedback)},
                                   allow_nan=False) + "\n")
            video.add(backend.capture_rgb()[0])
            return states, feedback

        for _ in range(settings.fps):
            advance("settle")

        def exercise(label, commands):
            print(f"R5_PHASE {label}", flush=True)
            before = robot.state()
            stable = 0
            # duration_s is a control tick, not a promise to finish within one tick.
            for step in range(settings.fps * 6):
                states, feedback = advance(label, commands if step == 0 else ())
                stable = stable + 1 if feedback.status == "completed" else 0
                if stable >= 5:
                    break
            for command in commands:
                if command.kind != "cartesian_pose":
                    continue
                target = robot.kinematics.world_pose(command.values, command.arm, command.frame)
                measured = states[command.arm].tcp_pose_world_wxyz
                position_error, orientation_error = pose_errors(measured, target)
                displacement = pose_errors(before[command.arm].tcp_pose_world_wxyz, measured)[0]
                check = {"phase": label, "arm": command.arm, "frame": command.frame,
                         "target_world_wxyz": target, "measured_world_wxyz": measured,
                         "position_error_m": position_error, "orientation_error_rad": orientation_error,
                         "displacement_m": displacement, "physics_duration_s": (step + 1) * config.control_dt_s,
                         "feedback": feedback.status,
                         "passed": stable >= 5 and position_error < 0.005 and orientation_error < 0.02
                                   and displacement > 0.01}
                result["pose_checks"].append(check)
            trace.flush()
            write_json(output / "result.json", result)

        # Reference joint vectors generate known-reachable poses only. The adapter
        # receives POSES and seeds IK from measured state; it never receives these vectors.
        references = ((0.15, 1.05, 1.3, 0.1, 0.15, -0.1),
                      (-0.15, 1.3, 1.15, -0.1, -0.15, 0.1))
        for arm in config.arms:
            for index, (reference, frame) in enumerate(zip(references, ("world", "robot_base"))):
                pose = robot.kinematics.forward(reference, arm, frame)
                exercise(f"{arm}_{frame}_{index}", (ArmCommand(arm, "cartesian_pose", pose, frame),))
        exercise("both_world", tuple(ArmCommand(arm, "cartesian_pose", robot.kinematics.forward(references[0], arm))
                                     for arm in config.arms))

        before_targets = dict(robot.targets)
        before_drives = {name: tuple(float(v) for v in arm.get_applied_action().joint_positions)
                         for name, arm in robot.instances.items()}
        bad = Action((ArmCommand("left", "gripper", (0.06,)),
                      ArmCommand("right", "cartesian_pose", (3., 0., 0.5, 1., 0., 0., 0.))), config.control_dt_s)
        rejected, reason = False, ""
        try:
            robot.apply(bad)
        except IKError as exc:
            rejected, reason = True, exc.result.reason
        after_drives = {name: tuple(float(v) for v in arm.get_applied_action().joint_positions)
                        for name, arm in robot.instances.items()}
        rejection = {"case": "valid_left_gripper_plus_unreachable_right_pose", "reason": reason,
                     "feedback": asdict(robot.feedback()),
                     "targets_unchanged": before_targets == robot.targets,
                     "physics_drives_unchanged": before_drives == after_drives,
                     "passed": rejected and before_targets == robot.targets and before_drives == after_drives
                               and robot.feedback().status == "rejected"}
        result["rejection_checks"].append(rejection)
        exercise("recover_home", tuple(ArmCommand(m.name, "cartesian_pose", robot.kinematics.forward(m.home_positions_rad, m.name))
                                       for m in config.robot.mounts))
        backend.reset(config.seed)
        robot.reset(config.seed)
        for _ in range(settings.fps):
            advance("after_reset")
        reset_error = max(abs(q - desired) for name, state in robot.state().items()
                          for q, desired in zip(state.joint_positions_rad, robot.initial_targets[name][:6]))
        result["reset_max_joint_error_rad"] = reset_error
        result["fk_vs_physx_max_position_error_m"] = max(error[0] for error in fk_errors)
        result["fk_vs_physx_max_orientation_error_rad"] = max(error[1] for error in fk_errors)
        result.update(status="completed", frames=video.count,
                      passed=all(c["passed"] for c in result["pose_checks"] + result["rejection_checks"])
                      and result["fk_vs_physx_max_position_error_m"] < 0.0005
                      and result["fk_vs_physx_max_orientation_error_rad"] < 0.002
                      and reset_error < 0.015 and not robot.cartesian_targets and robot.rejection is None
                      and video.count > 20 and video.maximum_pixel_change > 0.05)
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
