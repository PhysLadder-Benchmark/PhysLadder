"""Process-aware L2 scoring; force/strain limits are declared simulation proxies."""

import math

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import (
    held,
    quiet,
    relative,
    released,
    sustained,
    table_placed,
    upright,
)


def axial_quiet(row):
    """Supported round parts may spin about their own axis without relocating."""
    axis = pose_transform(row["pose"])[:3, 2]
    omega = np.asarray(row["angular_velocity"])
    return bool(
        np.linalg.norm(row["linear_velocity"]) < 0.025
        and np.linalg.norm(omega - (omega @ axis) * axis) < 0.4
    )


def marker(e, rows):
    p = e.spec.parameters
    body, cap = rows["body"], rows["cap"]
    b, c = pose_transform(body["pose"]), pose_transform(cap["pose"])
    tip = np.linalg.inv(c) @ b @ np.array([0.0, 0.0, p["body_tip_z"], 1.0])
    gap = float(p["cap_mouth_z"] - tip[2])
    assembly = np.linalg.inv(b) @ c
    # Native body/cap share a remote authoring origin. A small cap tilt can
    # separate those origins by >6 mm while the actual nib stays in the cavity.
    # Test the functional geometry, not agreement between USD root positions.
    tip_in_cap = bool(
        abs(assembly[2, 2]) > 0.95
        and np.linalg.norm(tip[:2]) < 0.0076
        and p["cap_mouth_z"] + 0.002 < tip[2] < p["cap_end_z"]
    )
    if (
        held(body, "left")
        and tip_in_cap
        and min(np.min(rows[n]["corners"], axis=0)[2] for n in ("body", "cap"))
        > p["table_z"] + 0.008
    ):
        e.flags.add("assembly_lifted")
    separated = gap >= 0.002
    if separated and held(body, "left") and held(cap, "right"):
        e.flags.add("separated_while_held")
    placed = {
        name: bool(
            released(row)
            and axial_quiet(row)
            and row["bench_force_n"] > p[name + "_mass_kg"] * 9.81 * 0.3
        )
        for name, row in (("body", body), ("cap", cap))
    }
    # A later arbitrary rotation must not fake recapping or undo an observed
    # separation. Detect actual insertion of either barrel end into cap cavity.
    ends = np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, p["body_tip_z"], 1.0]])
    local = ends @ (np.linalg.inv(c) @ b).T
    recapped = bool(
        abs(assembly[2, 2]) > 0.95
        and any(
            np.linalg.norm(v[:2]) < 0.0076
            and p["cap_mouth_z"] + 0.002 < v[2] < p["cap_end_z"]
            for v in local
        )
    )
    complete = (
        "assembly_lifted" in e.flags
        and "separated_while_held" in e.flags
        and all(placed.values())
        and not recapped
    )
    return (
        complete,
        {
            "assembly_lifted": "assembly_lifted" in e.flags,
            "tip_in_cap_cavity": tip_in_cap,
            "separated_while_held": "separated_while_held" in e.flags,
            "current_axial_gap_m": gap,
            "placed": placed,
            "recapped": recapped,
            "support_thresholds_n": {
                n: p[n + "_mass_kg"] * 9.81 * 0.3 for n in ("body", "cap")
            },
            "axial_spin_is_not_a_failure": True,
            "stages": {"capped_marker_lifted": "assembly_lifted" in e.flags,
                       "cap_removed_while_held": "separated_while_held" in e.flags,
                       "barrel_placed": "separated_while_held" in e.flags and placed["body"],
                       "cap_placed": "separated_while_held" in e.flags and placed["cap"]},
        },
        "",
    )


def books(e, rows):
    middle = rows["middle_book"]
    p = e.spec.parameters
    outward = np.asarray(p["outward_axis_world"])
    travel = float(
        (np.asarray(middle["pose"][:3]) - e.initial["middle_book"][:3]) @ outward
    )
    # The trailing corner must cross the fixed stack's exit plane. Projection
    # includes every corner, so translating or rotating sideways cannot fake it.
    rear_clearance = float(
        np.min((np.asarray(middle["corners"]) - p["stack_front_world"]) @ outward)
    )
    extracted = rear_clearance >= p["clearance_m"] and travel >= p["minimum_travel_m"]
    grasped = held(middle, e.spec.arms[0])
    if extracted and grasped:
        e.flags.add("middle_extracted_while_held")
    extracted_while_held = "middle_extracted_while_held" in e.flags
    table_supported = (
        middle["bench_force_n"] > 0.05
        and abs(float(np.min(np.asarray(middle["corners"])[:, 2])) - p["table_z"]) < 0.005
    )
    # Extraction is the goal. Do not force a regrasp of a book already resting
    # on the table, or add a stationary retention stage after a valid pull.
    complete = extracted and extracted_while_held and (grasped or table_supported)
    return (
        complete,
        {
            "middle_extracted": bool(extracted),
            "outward_travel_m": travel,
            "rear_clearance_m": rear_clearance,
            "outward_axis_world": outward.tolist(),
            "manipulation_arm": e.spec.arms[0],
            "held_by_manipulation_arm": grasped,
            "table_supported": bool(table_supported),
            "extracted_while_held": extracted_while_held,
            "adjacent_books": "fixed fixtures; no support evaluation",
            "release_required": False,
            "stages": {"grasped": "middle_book" in e.touched,
                       "fully_extracted": bool(extracted), "supported_after_extraction": bool(complete)},
        },
        "",
    )


def drawer(e, rows):
    row = rows["drawer"]
    p = e.spec.parameters
    # Rail limits and success use opening from fully CLOSED, not extra travel
    # from the pre-opened episode layout.
    displacement = np.asarray(row["pose"][:3]) - np.asarray(p["closed_pose"][:3])
    outward = np.asarray(p["outward_axis_world"])
    travel = float(displacement @ outward)
    guide_deviation = np.linalg.norm(
        displacement - travel * outward
    ) > 0.008 or not upright(row, 0.15)
    beyond_stop = travel < -0.006 or travel > p["max_travel_m"] + 0.006
    failure = ""
    if sustained(e, "drawer_guide_deviation", guide_deviation):
        failure = "Drawer guide constraint was persistently violated"
    if sustained(e, "drawer_beyond_stop", beyond_stop):
        failure = "Drawer persistently exceeded its mechanical stop"
    # Establish causal pull evidence during a continuous two-finger grasp,
    # independently of the final opening threshold. A last millimetre of passive
    # travel after release must not erase an otherwise measured grasped pull.
    grasped = held(row, p["manipulation_arm"]) and not guide_deviation and not beyond_stop
    pull = e.contact_motion.setdefault("drawer_pull", {"start": None, "net_m": 0.0})
    if grasped:
        if pull["start"] is None:
            pull["start"] = travel
        pull["net_m"] = max(pull["net_m"], travel - pull["start"])
        if pull["net_m"] >= p["minimum_grasped_pull_m"]:
            e.flags.add("pulled_while_held")
    else:
        pull["start"] = None
    at_stop = travel < 0.006 or travel > p["max_travel_m"] - 0.006
    impact = at_stop and np.linalg.norm(row["linear_velocity"]) > p["max_speed_m_s"]
    if impact:
        e.flags.add("drawer_speed_reference_exceeded")
    complete = (
        "pulled_while_held" in e.flags
        and p["required_open_m"] <= travel <= p["max_travel_m"] + 0.003
        and released(row)
        and quiet(row)
        and not guide_deviation
        and not beyond_stop
    )
    return (
        complete,
        {
            "opening_m": travel,
            "initial_opening_m": p["initial_open_m"],
            "travel_since_start_m": float((np.asarray(row["pose"][:3])
                                            - e.initial["drawer"][:3]) @ outward),
            "pulled_while_held": "pulled_while_held" in e.flags,
            "maximum_grasped_pull_m": pull["net_m"],
            "minimum_grasped_pull_m": p["minimum_grasped_pull_m"],
            "opening_reached": travel >= p["required_open_m"],
            "released": released(row),
            "quiet": bool(quiet(row)),
            "final_state": "released and open",
            "stages": {"grasped": "drawer" in e.touched,
                       "opened": "pulled_while_held" in e.flags and travel >= p["required_open_m"],
                       "released_open": bool(complete)},
            "quality_affects_success": False,
            "impact_speed_reference_m_s": p["max_speed_m_s"],
            "peak_speed_m_s": e.peak_speeds.get("drawer", 0.0),
            "quality_warnings": (
                ["Drawer speed at stop exceeded engineering reference"]
                if "drawer_speed_reference_exceeded" in e.flags
                else []
            ),
        },
        failure,
    )


from .pearl_pouring import score as pearl_pouring

RULES = {"L2-01": marker, "L2-03": books, "L2-04": pearl_pouring, "L2-07": drawer}
