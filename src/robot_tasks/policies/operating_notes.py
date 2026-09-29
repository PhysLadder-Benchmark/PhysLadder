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
    execution = ExecutionSettings.from_config(config)
    notes["gripper_completion"] = execution.gripper_completion
    if execution.gripper_completion == "stall_aware_v1":
        notes["gripper_feedback"] = (
            "closing_stalled means stable resistance, not a confirmed object grasp. "
            "Check RGB after a small test lift. completed means the command settled, not task success. "
            "An unchanged width is a hold request during arm motion; it will not reopen before lifting. "
            "If opening is blocked, the queued motion is cancelled. Choose an arm-only retreat "
            "to preserve the gripper drive target, or a separate gripper command. "
            "Close with a separate gripper-only command BEFORE requesting a lift."
        )
    notes["motion_mode"] = execution.motion_mode
    notes["startup_pose"] = execution.startup_pose
    if execution.startup_pose == "downward_ready_v1":
        notes["startup_preparation"] = (
            "Before your first observation the active arm moved to a fixed downward-facing ready pose. "
            "This is robot-only preparation, not a model action or a grasp. Use the measured TCP state."
        )
    if execution.motion_mode == "cartesian_linear_v1":
        notes["motion_execution"] = (
            "Requested Cartesian position and orientation are interpolated with bounded joint reference speeds. "
            "Known bench penetration or a discontinuous/unreachable path is rejected atomically. "
            "This is not obstacle-avoiding path planning: inspect RGB for objects and fixtures. "
            "After action_timeout choose a new action; queued motion has been cancelled."
        )
    if execution.gripper_timing == "open_before_motion":
        notes["gripper_sequence"] = (
            "For combined motion and a wider opening: open first while holding the arm, then move. "
            "Closing waits until motion settles. A gripper-only command executes immediately. "
            "A rejected combined action executes neither component."
        )
    return notes
