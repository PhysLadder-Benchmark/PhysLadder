"""Task-level ablations; provider credentials and model settings stay separate."""

from dataclasses import dataclass

from ..config import TaskConfig


@dataclass(frozen=True)
class PolicyOptions:
    retain_text_history: bool = False
    operating_notes: bool = False
    action_interface: str = "absolute_pose"
    camera_calibration: bool = False
    visual_aids: bool = False
    decision_notes: bool = False

    @classmethod
    def from_config(cls, config: TaskConfig) -> "PolicyOptions":
        data = config.raw.get("policy_options", {})
        if not isinstance(data, dict) or set(data) - cls.__dataclass_fields__.keys():
            raise ValueError("Unknown policy_options fields")
        result = cls(**data)
        for name in ("retain_text_history", "operating_notes", "camera_calibration", "visual_aids", "decision_notes"):
            if type(getattr(result, name)) is not bool:
                raise ValueError(f"policy_options.{name} must be boolean")
        if result.action_interface not in {"absolute_pose", "sparse_eef"}:
            raise ValueError("action_interface must be absolute_pose or sparse_eef")
        if result.action_interface == "sparse_eef" and not config.include_robot_state:
            raise ValueError("sparse_eef requires measured robot state to hold unspecified axes")
        if (result.camera_calibration or result.visual_aids) and not config.raw.get("simulation", {}).get("camera_profile"):
            raise ValueError("Camera calibration and visual aids require a camera rig profile")
        return result
