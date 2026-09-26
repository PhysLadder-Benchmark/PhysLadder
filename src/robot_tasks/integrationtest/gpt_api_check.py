"""One paid vision/schema request, without starting or moving a robot."""

from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from ..config import load_config
from ..policies.gpt import GPTPolicy, encode_frame
from ..policies.model_config import load_gpt_settings
from ..recording import write_json
from ..types import Feedback, Observation, PolicyInput, RGBFrame


def run_gpt_api_check(config_path: Path, model_path: Path, output_root: Path) -> tuple[Path, dict]:
    config, settings = load_config(config_path), load_gpt_settings(model_path)
    output = output_root / ("GPT_api_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
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
    write_json(output / "result.json", result)
    return output, result
