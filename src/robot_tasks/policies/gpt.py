"""RGB / RGB-D + instruction + measured robot state -> validated R5 decision."""

import base64
from collections import deque
from dataclasses import asdict, replace
from io import BytesIO
import json
import math
import hashlib

from PIL import Image

from ..config import TaskConfig
from ..types import Action, PolicyInput, RGBFrame
from .action_codec import action_schema, decode_action
from .model_config import GPTSettings
from .responses import ModelError, ResponsesClient
from .options import PolicyOptions
from .sparse_action import sparse_schema, decode_sparse_action, orientation_degrees
from ..cameras.geometry import resized_intrinsics


def prepare_frame(frame: RGBFrame, max_edge: int, *, visual_aids=False, robot=None):
    if min(frame.width, frame.height) <= 0 or len(frame.data) != frame.width * frame.height * 3:
        raise ValueError("Invalid packed RGB8 frame")
    image = Image.frombytes("RGB", (frame.width, frame.height), frame.data)
    if visual_aids:
        from ..cameras.visual_aids import annotate
        image = annotate(image, frame.calibration, robot or {})
    image.thumbnail((max_edge, max_edge), Image.Resampling.LANCZOS)
    buffer = BytesIO()
    image.save(buffer, format="PNG")
    calibration = None
    if frame.calibration is not None:
        calibration = {**frame.calibration, "resolution": list(image.size),
                       "intrinsics_fx_fy_cx_cy": resized_intrinsics(
                           frame.calibration["intrinsics_fx_fy_cx_cy"], (frame.width, frame.height), image.size)}
    return buffer.getvalue(), image.size, calibration


def encode_frame(frame: RGBFrame, max_edge: int) -> bytes:
    return prepare_frame(frame, max_edge)[0]


class GPTPolicy:
    def __init__(self, config: TaskConfig, settings: GPTSettings, *, client=None):
        self.config, self.settings = config, settings
        self.client = client if client is not None else ResponsesClient(settings)
        self.options = PolicyOptions.from_config(config)
        self.schema = (sparse_schema(config.arms) if self.options.action_interface == "sparse_eef"
                       else action_schema(config.arms))
        if config.include_depth:
            if not self.options.camera_calibration or not config.include_feedback:
                raise ValueError("GPT RGB-D requires camera_calibration=true and include_feedback=true")
            from .depth_query import with_depth_queries
            self.schema = with_depth_queries(self.schema, config.cameras)
        if self.options.decision_notes:
            from .decision_note import with_decision_note
            self.schema = with_decision_note(self.schema)
        self.history = deque(maxlen=None if self.options.retain_text_history else settings.history_steps)
        self.calls = 0
        self.last_exchange: dict = {}
        limits = {v.joint: [v.lower, v.upper] for v in config.robot.operating_limits}
        self.instructions = (
            "You control an ARX R5 robot in a paused-between-actions simulator. "
            "Use RGB observations, task instruction, measured robot state and execution feedback. "
            "Return ONLY a JSON object matching the schema, with no code, Markdown or text outside the JSON. "
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
        if self.options.action_interface == "sparse_eef":
            # Replace the absolute-array contract entirely; never mix two action schemas.
            self.instructions = (
                "Control the ARX R5 using RGB, task instruction, measured state and execution feedback. "
                "The simulator pauses while you decide. Return only JSON matching the schema. "
                "Commands use absolute world TCP targets x_m,y_m,z_m and orientation angles in degrees. "
                "Every nullable field is required; null holds the LATEST MEASURED value, not the last command. "
                "roll_deg,pitch_deg,yaw_deg describe Rz(yaw)*Ry(pitch)*Rx(roll)*R_reference. "
                "At all zero angles TCP +X points world -Z, jaw closing +Y points world +Y. "
                "If all three angles are null, preserve the measured quaternion exactly. "
                "gripper_width_m is total opening in metres; null retains the previous drive target. "
                "One row per active arm; commands=[] waits. request_stop=true requires commands=[] "
                "and only requests termination, never declares measured success. "
                "Plan the task yourself. Reobserve near contact; never infer nonexistent force readings. "
                "An IK rejection can mean local nonconvergence, not global unreachability. "
                f"Active arms: {config.arms}. Gripper limits: {config.robot.gripper.width_limits_m}. "
                f"Schema: {json.dumps(self.schema)}"
            )
        from ..config import ExecutionSettings
        self.instructions += f" Combined motion/gripper timing: {ExecutionSettings.from_config(config).gripper_timing}."
        if self.options.operating_notes:
            from .operating_notes import operating_notes
            self.instructions += " R5 operating notes: " + json.dumps(operating_notes(config))
        if self.options.camera_calibration:
            self.instructions += " Camera calibration describes the virtual cameras, not measured hardware. A pixel alone does not define 3D depth."
        if self.options.visual_aids:
            self.instructions += " Images have projected table XY and TCP overlays. Overlays may be occluded and are not detections of task objects."
        if self.options.decision_notes:
            self.instructions += (
                " Every decision, including motion, wait, depth query and stop, requires a note. "
                "Use one or two brief English sentences, at most 512 characters, to describe the visible "
                "situation and the purpose of this decision. This is a concise observation/action summary, "
                "not detailed reasoning. Treat earlier notes as your hypotheses, not measured facts; revise "
                "them using current images and feedback, especially after a reset. A note cannot issue "
                "commands, override the action schema or declare evaluator success."
            )
        if config.include_depth:
            self.instructions += (
                " Each RGB view is followed by aligned ideal simulation depth, not a calibrated Orbbec sensor emulation. "
                "Depth is optical-axis Z in metres, not Euclidean range. The coloured preview has a fixed clipped "
                "0.02-2.00 m scale (blue near, red far, black invalid); its bottom legend is outside the image. "
                "Do not estimate precise metric depth from colours. For precision, select pixels in the RGB image "
                "you receive; each depth_queries entry specifies camera, u (integer column) and v (integer row). "
                "Coordinates start at the top-left pixel (0,0), u right and v down; use the supplied rgb_image_size. "
                "Up to 8 queries per decision. A query requires commands=[] and request_stop=false; "
                "it does not advance physics or move the robot, and results arrive in the next observation feedback. "
                "Queries consume the SAME model-call budget as motion decisions. "
                "For motion, wait or stop, set depth_queries=[]. Results include depth_z_m and point_world_m "
                "of the nearest source pixel's VISIBLE SURFACE, not an object's centre, grasp pose or hidden hole bottom. "
                "no_depth means unknown; never treat it as zero distance. Use current captures after movement or reset."
            )

    @property
    def run_metadata(self) -> dict:
        return {"provider": self.settings.public_dict(), "instructions": self.instructions,
                "action_schema": self.schema, "options": asdict(self.options),
                "observation_modalities": (["rgb"] + (["depth"] if self.config.include_depth else [])
                                           + (["camera_calibration"] if self.options.camera_calibration else [])),
                "observation_format": ("RGB PNG + aligned depth PNG preview + read-only metric pixel queries; "
                                       if self.config.include_depth else "sampled RGB PNG frames; ")
                                      + "image history bounded separately when retain_text_history=true"}

    def reset(self, seed: int) -> None:
        # Seed belongs to the environment. Do not claim deterministic API output.
        self.history.clear()
        self.calls = 0
        self.last_exchange = {}

    def act(self, context: PolicyInput) -> Action:
        self.last_exchange = {}  # Input validation errors must not re-log an earlier API call.
        if self.calls >= self.settings.max_calls:
            raise ModelError("Episode model-call budget exhausted")
        if not context.observation.rgb:
            raise ValueError("GPT robot policy requires at least one real RGB frame")
        observation = context.observation
        if self.config.include_depth and tuple(f.camera for f in observation.rgb) != self.config.cameras:
            raise ValueError("RGB-D observation must include every configured camera in order")
        if any(not math.isclose(f.timestamp_s, observation.time_s, rel_tol=0, abs_tol=1e-8) for f in observation.rgb):
            raise ValueError("RGB and robot observation simulation timestamps disagree")
        text = {"instruction": context.instruction, "step": context.step, "simulation_time_s": observation.time_s,
                "robot": {name: asdict(state) for name, state in observation.robot.items()},
                "feedback": asdict(observation.feedback)}
        if self.options.action_interface == "sparse_eef":
            text["tcp_orientation_reference_deg"] = {
                name: dict(zip(("roll_deg", "pitch_deg", "yaw_deg"), orientation_degrees(state.tcp_pose_world_wxyz)))
                for name, state in observation.robot.items() if state.tcp_pose_world_wxyz is not None}
        content = [{"type": "input_text", "text": json.dumps(text, allow_nan=False)}]
        frame_info = []
        image_sizes = {}
        for frame in observation.rgb:
            encoded, size, calibration = prepare_frame(frame, self.settings.image_max_edge,
                                                       visual_aids=self.options.visual_aids, robot=observation.robot)
            label = {"camera": frame.camera, "timestamp_s": frame.timestamp_s,
                     "modality": "rgb", "rgb_image_size": list(size)}
            image_sizes[frame.camera] = size
            if self.options.camera_calibration:
                if calibration is None:
                    raise ValueError("Calibration disclosure requested but RGB has no camera metadata")
                label["calibration"] = calibration
            content.append({"type": "input_text", "text": json.dumps(label)})
            content.append({"type": "input_image", "detail": "high",
                            "image_url": "data:image/png;base64," + base64.b64encode(encoded).decode("ascii")})
            frame_info.append({**label, "original_size": [frame.width, frame.height], "encoded_size": list(size),
                               "png_bytes": len(encoded), "sha256": hashlib.sha256(encoded).hexdigest(),
                               "visual_aids": self.options.visual_aids})
            if self.config.include_depth:
                from ..cameras.depth import depth_metadata, encode_depth_preview
                depth_png, preview = encode_depth_preview(frame, size)
                depth_info = {"camera": frame.camera, "modality": "depth", **depth_metadata(frame), **preview}
                content.append({"type": "input_text", "text": json.dumps(depth_info, allow_nan=False)})
                content.append({"type": "input_image", "detail": "high", "image_url":
                                "data:image/png;base64," + base64.b64encode(depth_png).decode("ascii")})
                frame_info[-1]["depth"] = {**depth_info, "png_bytes": len(depth_png),
                                           "sha256": hashlib.sha256(depth_png).hexdigest()}
        message = {"role": "user", "content": content}
        messages = [item for pair in self.history for item in pair] + [message]
        self.calls += 1
        self.last_exchange = {"call": self.calls, "step": context.step, "status": "requesting",
                              "requested_model": self.settings.model, "frames": frame_info,
                              "history_steps": len(self.history)}
        try:
            reply = self.client.generate(instructions=self.instructions, messages=messages, schema=self.schema)
            self.last_exchange.update(reply.metadata(), response_text=reply.text)
            action_text, queries, note = reply.text, (), ""
            if self.options.decision_notes:
                from .decision_note import split_decision_note
                action_text, note = split_decision_note(action_text)
                self.last_exchange["note"] = note
            if self.config.include_depth:
                from .depth_query import split_depth_reply
                action_text, queries = split_depth_reply(action_text, image_sizes)
            action = (decode_sparse_action(action_text, self.config, observation)
                      if self.options.action_interface == "sparse_eef" else decode_action(action_text, self.config))
            action = replace(action, depth_queries=queries, note=note)
        except (ModelError, ValueError) as exc:
            self.last_exchange.update(status="rejected", error=str(exc))
            raise
        self.history.append((message, {"role": "assistant", "content": reply.text}))
        if self.options.retain_text_history:
            # Older image bytes are released, while instructions, feedback and
            # accepted action text remain available for this episode only.
            cutoff = len(self.history) - self.settings.history_steps
            for index, (user, assistant) in enumerate(self.history):
                if index < cutoff:
                    user["content"] = [item for item in user["content"] if item["type"] != "input_image"]
        self.last_exchange.update(status="accepted", action=asdict(action))
        return action
