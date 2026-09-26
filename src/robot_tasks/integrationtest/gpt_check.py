"""Live GPT -> R5 physics -> RGB feedback loop; no benchmark task score."""

from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from ..backends.isaacsim import IsaacSimBackend
from ..backends.settings import SimulationSettings
from ..config import load_config
from ..policies.gpt import GPTPolicy, encode_frame
from ..policies.model_config import load_gpt_settings, read_api_key
from ..recording import write_json
from ..robots.adapters import R5Adapter
from ..robots.audit import require_cartesian_sim_ready
from ..robots.transforms import pose_errors
from ..runtime import STORAGE_CONFIG
from ..types import Observation, PolicyInput
from .interaction_scene import InteractionScene, InteractionSettings
from .video import MotionVideo


def run_gpt_check(config_path: Path, output_root: Path, *, gui=False, run_directory=None):
    config = load_config(config_path)
    audit = require_cartesian_sim_ready(config.robot)
    if config.arms != ("left", "right") or not config.include_robot_state or not config.include_feedback:
        raise ValueError("GPT integration fixture requires both R5 arms, robot state and feedback")
    options = config.raw["gpt_check"]
    settings = load_gpt_settings(config.source.parent / options["model_config"])
    read_api_key(settings.api_key_env)  # Fail before launching Kit if credentials are missing.
    action_steps = options["max_action_steps"]
    if type(action_steps) is not int or not 1 <= action_steps <= config.max_steps:
        raise ValueError("max_action_steps must fit the integration step budget")
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    simulation = SimulationSettings.from_config(config)
    if not math.isclose(config.control_dt_s * simulation.fps, 1.0, abs_tol=1e-9):
        raise ValueError("GPT integration requires one video frame per control step")
    output = run_directory or output_root / ("R5_gpt_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("GPT-check output must be a new directory")
    (output / "config.toml").write_bytes(config.source.read_bytes())
    (output / "robot_profile.toml").write_bytes(config.robot.source.read_bytes())
    (output / "asset_source.lock.json").write_bytes(config.robot.asset_lock.read_bytes())
    (output / "storage_config.json").write_bytes(STORAGE_CONFIG.read_bytes())
    if simulation.camera_profile:
        (output / "camera_profile.toml").write_bytes((config.source.parent / simulation.camera_profile).read_bytes())
    write_json(output / "asset_audit.json", audit.as_dict())
    policy = GPTPolicy(config, settings)
    write_json(output / "model_config.json", settings.public_dict())
    (output / "system_prompt.txt").write_text(policy.instructions, encoding="utf-8")
    write_json(output / "manifest.json", {
        "evidence_type": "live_model_simulation_integration", "physics_during_inference": "paused",
        "action_execution": "hold absolute target until completion or per-action step budget",
        "instruction": config.instruction, "model_config": settings.public_dict(),
        "source_sha256": {str(p.relative_to(Path(__file__).parents[1])): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    backend, robot, video = IsaacSimBackend(), R5Adapter(), None
    result = {"status": "starting", "passed": False, "task_evaluated": False, "task_success": None,
              "scope": "RGB/API/R5 motion/feedback integration only; not contact-task performance",
              "model_calls": 0, "executed_actions": 0, "termination_reason": "starting"}
    write_json(output / "result.json", result)
    start = time.monotonic()
    trace = (output / "episode.jsonl").open("w", encoding="utf-8")
    exchanges = (output / "model.jsonl").open("w", encoding="utf-8")

    def budget():
        if time.monotonic() - start >= config.max_wall_time_s:
            raise TimeoutError("GPT integration wall-time budget exhausted")
        if video is not None and video.count >= config.max_steps:
            raise TimeoutError("GPT integration physics-step budget exhausted")

    def tick():
        budget()
        backend.step(config.control_dt_s)
        video.add(backend.capture_rgb()[0])

    def observe(step):
        observation = Observation(backend.time_s(), backend.capture_rgb(), robot.state(), robot.feedback())
        for frame in observation.rgb:
            (output / f"observation_{step:03d}_{frame.camera}.png").write_bytes(encode_frame(frame, settings.image_max_edge))
        return observation

    try:
        backend.open(config)
        robot.initialize(backend, config)
        scene = InteractionScene(backend, config, InteractionSettings.from_config(config))
        scene.reset("left")
        robot.reset(config.seed)
        policy.reset(config.seed)
        first_frame = backend.capture_rgb()[0]
        video = MotionVideo(output, replace(simulation, width=first_frame.width, height=first_frame.height))
        for _ in range(simulation.fps):
            tick()
        write_json(output / "scene.json", backend.scene_report)
        write_json(output / "import.json", backend.import_report)
        backend.save_scene(output / "scene.usda")
        initial = robot.state()
        target = list(initial["left"].tcp_pose_world_wxyz)
        target[2] += 0.02
        result["initial_robot"] = {k: asdict(v) for k, v in initial.items()}
        moved = opened = False
        result["termination_reason"] = "model_call_limit"
        for step in range(1, settings.max_calls + 1):
            budget()
            observation = observe(step)
            trace.write(json.dumps({"event": "observation", "step": step, "time_s": observation.time_s,
                                    "robot": {k: asdict(v) for k, v in observation.robot.items()},
                                    "feedback": asdict(observation.feedback)}, allow_nan=False) + "\n")
            trace.flush()
            print(f"R5_GPT_REQUEST {step}", flush=True)
            try:
                action = policy.act(PolicyInput(config.instruction, observation, step))
            finally:
                exchanges.write(json.dumps(policy.last_exchange, allow_nan=False) + "\n")
                exchanges.flush()
                result["model_calls"] = policy.calls
            budget()  # A late response must not start any motion.
            if action.request_stop:
                result["termination_reason"] = "policy_requested_stop"
                break
            robot.apply(action)  # R5 adapter checks both arms atomically, including IK feasibility.
            result["executed_actions"] += bool(action.commands)
            stable = 0
            for _ in range(action_steps):
                tick()
                feedback = robot.feedback()
                if feedback.status in {"error", "rejected"}:
                    raise RuntimeError("R5 rejected model action")
                stable = stable + 1 if feedback.status == "completed" else 0
                if stable >= 3:
                    break
            states = robot.state()
            p_error, r_error = pose_errors(states["left"].tcp_pose_world_wxyz, target)
            moved = moved or (p_error <= 0.005 and r_error <= 0.02)
            opened = opened or (moved and abs(states["left"].gripper_width_m - 0.06) <= 0.002)
            trace.write(json.dumps({"event": "execution", "step": step, "action": asdict(action),
                                    "time_s": backend.time_s(), "feedback": asdict(robot.feedback()),
                                    "robot": {k: asdict(v) for k, v in states.items()}}, allow_nan=False) + "\n")
            trace.flush()
            print(f"R5_GPT_EXECUTED {step} feedback={robot.feedback().status}", flush=True)
        observe(policy.calls + 1)
        final = robot.state()
        right_drift = max(abs(a-b) for a, b in zip(final["right"].joint_positions_rad, initial["right"].joint_positions_rad))
        result.update(status="completed", passed=(moved and opened and right_drift < 0.01
                                                   and result["termination_reason"] == "policy_requested_stop"
                                                   and policy.calls >= 2),
                      measured_checks={"left_tcp_raised_20mm": moved, "left_gripper_opened_60mm": opened,
                                       "right_joint_drift_rad": right_drift},
                      final_robot={k: asdict(v) for k, v in final.items()})
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="error", passed=False, termination_reason="exception",
                      error=f"{type(exc).__name__}: {exc}")
        print(f"R5_GPT_ERROR {result['error']}", flush=True)
    finally:
        trace.close()
        exchanges.close()
        if video is not None:
            try:
                video.close()
            except Exception as exc:
                result.update(status="error", passed=False, video_error=str(exc))
            result["video_frames"] = video.count
        result["wall_time_s"] = time.monotonic() - start
        result["simulation_time_s"] = backend.time_s() if backend.world is not None else 0
        write_json(output / "result.json", result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        backend.close()
    return output, result
