"""One paid vision/schema request, without starting or moving a robot."""

from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from uuid import uuid4

from ..config import load_config
from ..policies.gpt import GPTPolicy, encode_frame
from ..policies.model_config import load_gpt_settings
from ..recording import finish_run, write_json
from ..types import Feedback, Observation, PolicyInput, RGBFrame


def run_gpt_api_check(config_path: Path, output_root: Path, *, model_path: Path | None = None,
                      gui: bool = False, run_directory: Path | None = None) -> tuple[Path, dict]:
    if gui:
        raise ValueError("API-only check does not start a simulation GUI")
    config = load_config(config_path)
    settings = load_gpt_settings(model_path if model_path is not None
                                 else config.source.parent / config.raw["gpt_check"]["model_config"])
    output = run_directory or output_root / ("GPT_api_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=run_directory is not None)
    if any(p.name != "process.log" for p in output.iterdir()):
        raise ValueError("API-check output must be a new directory")
    policy = GPTPolicy(config, settings)
    # A declared synthetic fixture, never represented as a simulated robot image.
    frame = RGBFrame("color_fixture", 0.0, 64, 64, bytes((240, 10, 10)) * (64 * 64))
    (output / "input.png").write_bytes(encode_frame(frame, settings.image_max_edge))
    instruction = ("This is a vision/API fixture; no physical robot will execute it. "
                   "If the image is predominantly red, command left gripper width 0.06 m; "
                   "if blue, command left gripper width 0.02 m. Only one gripper command, frame=world, request_stop=false.")
    write_json(output / "request_summary.json", {"model": settings.public_dict(), "instruction": instruction,
                                                "synthetic_image": True, "robot_execution": False})
    result = {"status": "error", "passed": False, "task_evaluated": False, "task_success": None}
    try:
        action = policy.act(PolicyInput(instruction, Observation(0, (frame,), {}, Feedback("not_initialized")), 1))
        passed = (not action.request_stop and len(action.commands) == 1
                  and action.commands[0].arm == "left" and action.commands[0].kind == "gripper"
                  and action.commands[0].values == (0.06,))
        result.update(status="completed", passed=passed, action=asdict(action))
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    write_json(output / "model_exchange.json", policy.last_exchange)
    finish_run(output, result)
    print("R5_API_RESULT " + json.dumps({"output": str(output), **result}, ensure_ascii=False), flush=True)
    return output, result
