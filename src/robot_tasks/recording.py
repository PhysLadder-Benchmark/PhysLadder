"""Persistent episode evidence. No GIF/video is fabricated by the scaffold."""

import hashlib
import json
import platform
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from .config import TaskConfig
from .types import Observation


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


class EpisodeRecorder:
    def __init__(self, root: Path, config: TaskConfig, mode: str, step_limit: int):
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.directory = root / f"{config.task_id}_{mode}_{name}_{uuid4().hex[:8]}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.save_rgb = config.save_rgb
        config_bytes = config.source.read_bytes()
        (self.directory / "config.toml").write_bytes(config_bytes)
        robot_profile_bytes = config.robot.source.read_bytes()
        (self.directory / "robot_profile.toml").write_bytes(robot_profile_bytes)
        source_lock_bytes = config.robot.asset_lock.read_bytes()
        (self.directory / "asset_source.lock.json").write_bytes(source_lock_bytes)
        package = Path(__file__).parent
        sources = {
            str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(package.rglob("*.py"))
        }
        write_json(self.directory / "manifest.json", {
            "task_id": config.task_id, "mode": mode,
            "evidence_type": "framework_dry_run" if mode == "dry_run" else "simulation_attempt",
            "mock": mode == "dry_run", "seed": config.seed, "step_limit": step_limit,
            "config_path": str(config.source), "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
            "robot_asset": str(config.asset_path) if config.asset_path else None,
            "robot_profile_path": str(config.robot.source),
            "robot_profile_sha256": hashlib.sha256(robot_profile_bytes).hexdigest(),
            "asset_source_lock_sha256": hashlib.sha256(source_lock_bytes).hexdigest(),
            "available_arms": config.robot.available_arms, "active_arms": config.arms,
            "python": platform.python_version(), "platform": platform.platform(),
            "source_sha256": sources,
            "units": {"distance": "m", "angle": "rad", "time": "s", "quaternion": "wxyz"},
            "video_status": "not_implemented",
        })

    def observation(self, observation: Observation, step: int) -> dict:
        frames = []
        for frame in observation.rgb:
            if len(frame.data) != frame.width * frame.height * 3 or min(frame.width, frame.height) <= 0:
                raise ValueError("Invalid packed RGB8 frame")
            saved_path = None
            if self.save_rgb:
                # Camera names are sanitized independently of adapter inputs.
                if not frame.camera.replace("_", "").isalnum():
                    raise ValueError("Invalid camera name")
                relative = Path("frames") / frame.camera / f"{step:06d}.ppm"
                target = self.directory / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                header = f"P6\n{frame.width} {frame.height}\n255\n".encode("ascii")
                target.write_bytes(header + frame.data)
                saved_path = relative.as_posix()
            frames.append({"camera": frame.camera, "time_s": frame.timestamp_s,
                           "width": frame.width, "height": frame.height, "path": saved_path})
        return {"time_s": observation.time_s, "rgb": frames,
                "robot": {k: asdict(v) for k, v in observation.robot.items()},
                "feedback": asdict(observation.feedback)}

    def event(self, value: dict) -> None:
        with (self.directory / "episode.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")

    def finish(self, result: dict) -> None:
        write_json(self.directory / "result.json", result)

    def export_video(self) -> None:
        # TODO: encode real RGB frames, keeping actual time and camera metadata.
        raise NotImplementedError("GIF/MP4 export is reserved for real task rendering")
