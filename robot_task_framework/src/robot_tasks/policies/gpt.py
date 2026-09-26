"""RGB + instruction + measured robot state -> validated R5 action."""

import base64
from collections import deque
from dataclasses import asdict
from io import BytesIO
import json

from PIL import Image

from ..config import TaskConfig
from ..types import Action, PolicyInput, RGBFrame
from .action_codec import action_schema, decode_action
from .model_config import GPTSettings
from .responses import ModelError, ResponsesClient


def encode_frame(frame: RGBFrame, max_edge: int) -> bytes:
    if min(frame.width, frame.height) <= 0 or len(frame.data) != frame.width * frame.height * 3:
        raise ValueError("Invalid packed RGB8 frame")
    image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
    image.thumbnail((max_edge, max_edge), Image.Resampling.LANCZOS)
    buffer = BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


class GPTPolicy:
    def __init__(self, config: TaskConfig, settings: GPTSettings, *, client=None):
        self.config, self.settings = config, settings
        self.client = client if client is not None else ResponsesClient(settings)
        self.schema = action_schema(config.arms)
        self.history = deque(maxlen=settings.history_steps)
        self.calls = 0
        self.last_exchange: dict = {}
        limits = {v.joint: [v.lower, v.upper] for v in config.robot.operating_limits}
        self.instructions = (
            "You control an ARX R5 robot in a paused-between-actions simulator. "
            "Use RGB observations, task instruction, measured robot state and execution feedback. "
            "Return ONLY a JSON object matching the schema. Do not produce code, Markdown or commentary. "
            "Commands are absolute targets. joint_position: six joint angles in radians in joint order. "
            "cartesian_pose: seven numbers [x,y,z,qw,qx,qy,qz], metres and normalized quaternion, "
            "world or robot_base frame. gripper: one total opening width in metres. "
            "Write action numbers as fixed-point decimals rounded to 6 decimal places, not scientific notation. "
            "When preserving orientation, copy all four quaternion components with their correct magnitudes; "
            "for example a component 3.1e-06 becomes 0.000003, not 3.1. "
            "World Z points up. TCP local +X points forward, +Y is finger closing axis. "
            "One motion command and one gripper command maximum per arm per decision. "
            "An empty commands list means wait/observe while keeping existing targets. "
            "Avoid large motion jumps: IK does not provide collision avoidance or straight-line paths. "
            "request_stop=true must have no commands; this requests termination, not a success label. "
            "Do not infer exact object coordinates or force values from nonexistent sensors. "
            f"Joint order: {config.robot.joint_names}. Limits: {json.dumps(limits)}. "
            f"Gripper width range: {config.robot.gripper.width_limits_m}. "
            f"Active arms: {config.arms}. Schema: {json.dumps(self.schema)}"
        )

    @property
    def run_metadata(self) -> dict:
        return {"provider": self.settings.public_dict(), "instructions": self.instructions,
                "action_schema": self.schema, "observation_format": "sampled RGB PNG frames and bounded history"}

    def reset(self, seed: int) -> None:
        # Seed belongs to the environment. Do not claim deterministic API output.
        self.history.clear()
        self.calls = 0
        self.last_exchange = {}

    def act(self, context: PolicyInput) -> Action:
        if self.calls >= self.settings.max_calls:
            raise ModelError("Episode model-call budget exhausted")
        if not context.observation.rgb:
            raise ValueError("GPT robot policy requires at least one real RGB frame")
        observation = context.observation
        text = {"instruction": context.instruction, "step": context.step, "simulation_time_s": observation.time_s,
                "robot": {name: asdict(state) for name, state in observation.robot.items()},
                "feedback": asdict(observation.feedback)}
        content = [{"type": "input_text", "text": json.dumps(text, allow_nan=False)}]
        frame_info = []
        for frame in observation.rgb:
            encoded = encode_frame(frame, self.settings.image_max_edge)
            label = {"camera": frame.camera, "timestamp_s": frame.timestamp_s}
            content.append({"type": "input_text", "text": json.dumps(label)})
            content.append({"type": "input_image", "detail": "high",
                            "image_url": "data:image/png;base64," + base64.b64encode(encoded).decode("ascii")})
            frame_info.append({**label, "original_size": [frame.width, frame.height], "png_bytes": len(encoded)})
        message = {"role": "user", "content": content}
        messages = [item for pair in self.history for item in pair] + [message]
        self.calls += 1
        self.last_exchange = {"call": self.calls, "step": context.step, "status": "requesting",
                              "requested_model": self.settings.model, "frames": frame_info,
                              "history_steps": len(self.history)}
        try:
            reply = self.client.generate(instructions=self.instructions, messages=messages, schema=self.schema)
            self.last_exchange.update(reply.metadata(), response_text=reply.text)
            action = decode_action(reply.text, self.config)
        except (ModelError, ValueError) as exc:
            self.last_exchange.update(status="rejected", error=str(exc))
            raise
        self.history.append((message, {"role": "assistant", "content": reply.text}))
        self.last_exchange.update(status="accepted", action=asdict(action))
        return action
