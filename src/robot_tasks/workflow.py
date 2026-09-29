"""One fixed GPT workflow; TOML only supplies experiment parameters."""

from copy import deepcopy
from pathlib import Path
import tomllib


SECTIONS = {"runtime", "observation", "recording", "execution", "policy"}


def workflow_metadata() -> dict:
    """Describe implemented behavior for inspection and run records, not switches."""
    return {
        "version": 3, "action_interface": "sparse_eef", "ik": "lula",
        "observation_modalities": ["rgb", "depth", "camera_calibration", "robot_state", "feedback"],
        "retain_text_history": True, "operating_notes": True, "visual_aids": True,
        "decision_notes": True, "gripper_timing": "after_motion",
        "active_observation": "according_to_task_arm_roles",
        "startup_preparation": "downward_ready_v1",
        "observation_preparation": "face_manipulation_workspace_v1",
    }


def _fixed_inputs(raw: dict) -> dict:
    if "policy_options" in raw:
        raise ValueError("Remove [policy_options]: GPT workflow capabilities are always enabled")
    if "gripper_timing" in raw.get("execution", {}):
        raise ValueError("Remove execution.gripper_timing: motion always precedes the gripper command")
    result = deepcopy(raw)
    observation = result.setdefault("observation", {})
    for key in ("include_depth", "include_robot_state", "include_feedback"):
        if key in observation and observation[key] is not True:
            raise ValueError(f"observation.{key} is required by the fixed GPT workflow; remove this override")
        observation[key] = True
    return result


def validate_gpt_workflow(config) -> None:
    """Also validate programmatic callers before constructing a paid request."""
    _fixed_inputs(config.raw)
    if not (config.include_depth and config.include_robot_state and config.include_feedback):
        raise ValueError("GPT workflow requires RGB-D, measured robot state and execution feedback")
    if not config.raw.get("simulation", {}).get("camera_profile"):
        raise ValueError("GPT workflow requires a camera rig profile with calibration")


def load_workflow(source: Path, task: dict) -> tuple[dict, Path | None, dict | None]:
    """Task fields override profile fields. A profile cannot define task assets/roles.

    Keep this to one explicit profile, without recursive inheritance or hidden defaults.
    File references remain in the task so their base directory is unambiguous.
    """
    reference = task.get("workflow_profile")
    if reference is None:
        return _fixed_inputs(task), None, None
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("workflow_profile must reference a TOML file")
    path = (source.parent / reference).resolve()
    with path.open("rb") as stream:
        profile = tomllib.load(stream)
    if not profile or set(profile) - SECTIONS:
        raise ValueError("Workflow profiles may only define runtime/observation/recording/execution/policy parameters")
    if any(not isinstance(v, dict) for v in profile.values()):
        raise ValueError("Workflow profile sections must be tables")
    if set(profile.get("policy", {})) - {"reasoning_effort"}:
        raise ValueError("Workflow policy only configures reasoning_effort; model_config belongs to the task")
    # Validate both sources so a task cannot mask obsolete profile switches.
    _fixed_inputs(profile)
    _fixed_inputs(task)
    merged = deepcopy(profile)
    for section, value in task.items():
        if section in profile:
            if not isinstance(value, dict):
                raise ValueError(f"Task override {section} must be a table")
            merged[section].update(deepcopy(value))
        else:
            merged[section] = deepcopy(value)
    return _fixed_inputs(merged), path, profile
