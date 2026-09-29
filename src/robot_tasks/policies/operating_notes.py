"""Configuration-derived robot facts. Never include task-object ground truth."""

from ..config import ExecutionSettings
from ..robots.kinematics import R5Kinematics
from ..backends.scene_geometry import BENCH_POSITION, BENCH_SIZE, BENCH_TOP


def operating_notes(config) -> dict:
    profile = config.robot
    kinematics = R5Kinematics(profile)
    notes = {
        "units": "metres, radians unless explicitly labelled degrees; quaternion wxyz",
        "world_axes": "+Z up; +X forward from the configured bases; +Y toward the left base",
        "base_poses_world_wxyz": {m.name: m.base_pose_wxyz for m in profile.mounts},
        "bench": {"centre_m": BENCH_POSITION, "size_m": BENCH_SIZE, "surface_world_z_m": BENCH_TOP},
        "tcp": {"pose_in_link6_wxyz": profile.tcp_pose_wxyz, "source": profile.tcp_source,
                "meaning": "Control point between distal fingers. Command this point directly; do not add a tool offset.",
                "axes": "+X forward along fingers; +Y jaw closing; +Z transverse"},
        "gripper": {"total_width_limits_m": profile.gripper.width_limits_m,
                    "smaller_width": "closes fingers", "larger_width": "opens fingers",
                    "calibration_source": profile.gripper.calibration_source},
        "conservative_reach_bound_m": kinematics.reach_bound_m,
        "reach_warning": "Bound is necessary, not sufficient. A local IK failure does not prove global unreachability.",
        "gripper_timing": ExecutionSettings.from_config(config).gripper_timing,
        "contact": "Use measured TCP and RGB to reobserve. A blocked closing target is not proof of a grasp. No force sensor is provided.",
        "information_boundary": "Robot configuration only; no object pose, grasp point, task score, or successful trajectory.",
    }
    camera_path = config.raw.get("simulation", {}).get("camera_profile")
    if camera_path:
        from ..cameras.profile import load_camera_rig
        notes["camera_roles"] = {c.name: {"model": c.model, "parent": c.parent}
                                 for c in load_camera_rig(config.source.parent / camera_path).select(config.cameras)}
    notes["motion_backend"] = "ik"
    notes["startup_preparation"] = (
        "Before your first observation the manipulation arm moved to a fixed downward-facing ready pose. "
        "This is robot-only preparation, not a model action or a grasp. Use the measured TCP state. "
        "Observation arms start with their wrist cameras facing the manipulation workspace. "
        "You may reposition them for a clearer view using the same motion interface."
    )
    return notes
