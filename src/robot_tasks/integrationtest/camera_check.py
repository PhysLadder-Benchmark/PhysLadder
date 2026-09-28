"""Three-view R5 camera check; provisional calibration, no benchmark score."""

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import traceback
from uuid import uuid4

import numpy as np
from PIL import Image

from ..backends.isaacsim import IsaacSimBackend
from ..backends.settings import SimulationSettings
from ..config import load_config
from ..recording import write_json
from ..robots.adapters import R5Adapter
from ..robots.audit import require_cartesian_sim_ready
from ..robots.transforms import pose_errors, pose_transform, transform_pose
from ..types import Action, ArmCommand
from ..tasks.pick_place_scene import InteractionScene, InteractionSettings
from ..video import MotionVideo


def run_camera_check(config_path: Path, output_root: Path, *, gui=False, run_directory=None):
    config = load_config(config_path)
    require_cartesian_sim_ready(config.robot)
    if config.cameras != ("overview", "left_wrist", "right_wrist"):
        raise ValueError("Camera check requires overview, left_wrist and right_wrist in this order")
    if gui:
        config = replace(config, raw={**config.raw, "simulation": {**config.raw["simulation"], "headless": False}})
    settings = SimulationSettings.from_config(config)
    if not settings.camera_profile:
        raise ValueError("Camera check requires a camera_profile")
    output = run_directory or output_root / ("R5_cameras_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("Camera-check output must be a new directory")
    print(f"R5_CHECK_OUTPUT {output}", flush=True)
    for source, dest in ((config.source, "config.toml"), (config.robot.source, "robot_profile.toml"),
                         (config.source.parent / settings.camera_profile, "camera_profile.toml")):
        (output / dest).write_bytes(source.read_bytes())
    write_json(output / "manifest.json", {
        "kind": "three_camera_integration", "hardware_calibrated": False, "gpt_called": False,
        "source_sha256": {p.relative_to(Path(__file__).parents[1]).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in Path(__file__).parents[1].rglob("*.py")},
    })
    result = {"status": "starting", "passed": False, "task_evaluated": False, "hardware_calibrated": False,
              "scope": "R5-mounted RGB cameras, physical attachment, synchronized capture and model input packing"}
    backend, robot, videos = IsaacSimBackend(), R5Adapter(), {}
    start = time.monotonic()
    trace = (output / "camera_poses.jsonl").open("w", encoding="utf-8")
    errors = {name: [] for name in config.cameras}
    image_std = {name: [] for name in config.cameras}
    snapshots = {}
    try:
        backend.open(config)
        robot.initialize(backend, config)
        scene = InteractionScene(backend, config, InteractionSettings.from_config(config))
        scene.reset("left")
        robot.reset(config.seed)
        for _ in range(3):
            backend.capture_rgb()
        for spec in backend.camera_rig.specs:
            directory = output / spec.name
            directory.mkdir()
            videos[spec.name] = MotionVideo(directory, replace(settings, width=spec.resolution[0], height=spec.resolution[1]))

        def sample(label):
            if time.monotonic() - start > config.max_wall_time_s:
                raise TimeoutError("Camera check exceeded its time budget")
            frames = backend.capture_rgb()
            if tuple(f.camera for f in frames) != config.cameras or len({f.timestamp_s for f in frames}) != 1:
                raise RuntimeError("RGB camera ordering/timestamps do not match the observation contract")
            poses = backend.camera_rig.world_poses()
            for spec, frame in zip(backend.camera_rig.specs, frames):
                parent = np.eye(4) if spec.parent == "world" else pose_transform(backend.read_ee_pose(spec.parent))
                expected = transform_pose(parent @ pose_transform(spec.optical_pose_wxyz))
                errors[spec.name].append(pose_errors(poses[spec.name], expected))
                image_std[spec.name].append(float(np.frombuffer(frame.data, np.uint8).std()))
                videos[spec.name].add(frame)
            trace.write(json.dumps({"phase": label, "time_s": backend.time_s(), "poses": poses}) + "\n")
            return poses, frames

        for _ in range(settings.fps):
            backend.step(config.control_dt_s)
            poses, frames = sample("settle")
        snapshots["initial"] = poses
        for arm, sign in (("left", 1), ("right", -1)):
            target = list(robot.initial_targets[arm][:6])
            target[0] += sign * 0.18
            robot.apply(Action((ArmCommand(arm, "joint_position", tuple(target)),), config.control_dt_s))
            for _ in range(settings.fps * 2):
                backend.step(config.control_dt_s)
                poses, frames = sample(arm + "_motion")
            snapshots[arm + "_moved"] = poses

        # Exercise the actual model-input assembler without making a paid API call.
        from ..policies.gpt import GPTPolicy
        from ..policies.model_config import GPTSettings
        from ..policies.responses import ModelReply
        from ..types import Observation, PolicyInput

        class CaptureClient:
            def generate(self, **kwargs):
                self.messages = kwargs["messages"]
                return ModelReply(text='{"commands":[],"request_stop":true}', model="fixture",
                                  response_id="camera_check", usage={}, latency_s=0.0)

        client = CaptureClient()
        policy = GPTPolicy(config, GPTSettings("https://example.invalid", "fixture"), client=client)
        policy.act(PolicyInput(config.instruction, Observation(backend.time_s(), frames, robot.state(), robot.feedback()), 1))
        last_input = client.messages[-1]["content"]
        image_count = sum(item["type"] == "input_image" for item in last_input)
        result["gpt_input"] = {"image_count": image_count, "frames": policy.last_exchange["frames"], "api_called": False}
        checks = {}
        for name in config.cameras:
            motion = pose_errors(snapshots["initial"][name], snapshots["right_moved"][name])[0]
            max_position = max(pair[0] for pair in errors[name])
            max_rotation = max(pair[1] for pair in errors[name])
            checks[name] = {
                "mount_position_error_m": max_position, "mount_rotation_error_rad": max_rotation,
                "position_change_m": motion, "minimum_rgb_std": min(image_std[name]),
                "maximum_pixel_change": videos[name].maximum_pixel_change, "frames": videos[name].count,
                "passed": max_position < 1e-4 and max_rotation < 1e-3 and min(image_std[name]) > 3
                          and videos[name].maximum_pixel_change > 0.1
                          and (motion < 1e-5 if name == "overview" else motion > 0.01),
            }
        stationary = pose_errors(snapshots["initial"]["right_wrist"], snapshots["left_moved"]["right_wrist"])[0]
        result.update(status="completed", cameras=checks, unmoved_wrist_drift_m=stationary,
                      passed=all(c["passed"] for c in checks.values()) and stationary < 0.001 and image_count == 3)
        write_json(output / "scene.json", backend.scene_report)
        backend.save_scene(output / "scene.usda")
        # A labelled preview; raw per-camera PNGs and videos retain their original sizes.
        from PIL import ImageDraw
        preview = Image.new("RGB", (640 * 3, 520), "#182028")
        draw = ImageDraw.Draw(preview)
        for index, frame in enumerate(frames):
            preview.paste(Image.frombytes("RGB", (frame.width, frame.height), frame.data), (index * 640, 32))
            draw.text((index * 640 + 10, 10), frame.camera + " | provisional / uncalibrated", fill="white")
        preview.save(output / "three_views.png")
    except (Exception, KeyboardInterrupt) as exc:
        result.update(status="error", passed=False, error=f"{type(exc).__name__}: {exc}")
        (output / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        print(f"R5_CAMERA_ERROR {exc}", flush=True)
    finally:
        trace.close()
        for video in videos.values():
            try:
                video.close()
            except Exception as exc:
                result.update(status="error", passed=False, video_error=str(exc))
        result["wall_time_s"] = time.monotonic() - start
        write_json(output / "result.json", result)
        print("R5_CHECK_RESULT " + json.dumps(result), flush=True)
        backend.close()
    return output, result
