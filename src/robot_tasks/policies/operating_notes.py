"""Configuration-derived robot facts. Never include task-object ground truth."""

from ..config import ExecutionSettings
from ..robots.kinematics import R5Kinematics
from ..backends.scene_geometry import BENCH_POSITION, BENCH_SIZE, BENCH_TOP


def manipulation_guidance(config) -> str:
    """Shared operating advice, independent of the task and object layout."""
    opening = config.robot.gripper.width_limits_m[1]
    grasp_advice = (
        "For an engaged part, verify contact through images and small task-appropriate movements along or about its axis; do not bend it with a vertical test lift. "
        if config.grasp_check_mode == "contact_inspection" else
        "Verify a grasp from object motion during a short lift and from current images and feedback. "
    )
    return (
        "Use the latest measured state after preparation, execution, timeout or reset. "
        "Read all available views before moving near uncertain geometry. An empty hole pixel may show "
        "the background or a distant surface; query visible rim or object surfaces and account for "
        "their geometry instead of treating one depth point as an object's centre. "
        "Use an observation arm when needed, following its role restrictions.\n\n"
        "Choose grips that leave functional ends and the later approach path clear. Check clearance "
        "for the complete fingers and carried object, not only the TCP. A held object's centre, axis "
        "and working tip may be offset from the TCP; estimate that relationship from the actual grasp "
        "and use it when choosing subsequent TCP targets. No object-to-hand attachment is provided.\n\n"
        f"When an open grasp approach is needed, set gripper=1 (physical opening {opening:.3f} m) "
        "in a separate action BEFORE approaching: a combined command moves the arm first. "
        "Use gripper=0 to close around an object with the configured effort cap. The fingers stop where physical contact permits; the actual measured opening can remain larger than the empty closing target. Do not widen merely to eliminate this expected width error. "
        f"{grasp_advice}"
        "A completed gripper command or a blocked closing target alone is not grasp evidence. "
        "Recheck that the object is still held before every transport or insertion decision. "
        "If it slips, falls or is no longer visible between the fingers, do not continue the old "
        "insertion plan or assume its previous offset from the TCP is valid. Reobserve and "
        "re-establish the grasp, or follow an attempt-reset notice if one is provided.\n\n"
        "Move efficiently in clear free space; near contact use millimetre-scale corrections and "
        "reobserve. Before transporting, ensure the lowest part of the payload clears intervening "
        "surfaces. The planner checks approximate robot/table/fixture geometry and measured held-object clearance. This is planner-only privileged geometry, not task reasoning or proof of a grasp; moving unheld objects are excluded to allow task contact. A planned path can still deviate under physical contact. Large simultaneous "
        "translation and rotation requests can fail; intermediate reachable targets or separated "
        "orientation changes may help. Never assume the commanded target was achieved.\n\n"
        "If progress stops or unexpected tilt/contact appears, inspect the views and feedback, "
        "withdraw to a clear pose when appropriate, and revise alignment or grasp. Do not repeatedly "
        "send the same blocked target or invent force measurements or unsupported force commands. "
        "After release, withdraw without dragging the object and verify the task's required stable "
        "support and duration using simulation timestamps. Wait actions hold existing drive targets; "
        "ensure those targets are appropriate before waiting. Stop only according to the task goal "
        "and the declared termination contract."
    )



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
                    "command": "gripper in [0,1]: 0 closes, 1 opens; null retains drive",
                    "calibration_source": profile.gripper.calibration_source},
        "conservative_reach_bound_m": kinematics.reach_bound_m,
        "reach_warning": "Bound is necessary, not sufficient. A local IK failure does not prove global unreachability.",
        "gripper_timing": ExecutionSettings.from_config(config).gripper_timing,
        "contact": "Use measured TCP and RGB to reobserve. A blocked closing target is not proof of a grasp. No force sensor is provided.",
        "information_boundary": "Robot configuration only; no object pose, grasp point, detailed task score, or successful trajectory. A rejected stop discloses only that task completion was not confirmed.",
        "manipulation_guidance": manipulation_guidance(config),
    }
    camera_path = config.raw.get("simulation", {}).get("camera_profile")
    if camera_path:
        from ..cameras.profile import load_camera_rig
        notes["camera_roles"] = {c.name: {"model": c.model, "parent": c.parent}
                                 for c in load_camera_rig(config.source.parent / camera_path).select(config.cameras)}
    notes["motion_backend"] = "curobo_trajectory"
    notes["startup_preparation"] = (
        "Before your first observation the manipulation arm moved to a fixed downward-facing ready pose. "
        "This is robot-only preparation, not a model action or a grasp. Use the measured TCP state. "
        "Observation arms start with their wrist cameras facing the manipulation workspace. "
        "You may reposition them for a clearer view using the same motion interface."
    )
    if config.initial_state_description:
        notes["startup_preparation"] = config.initial_state_description
    return notes
