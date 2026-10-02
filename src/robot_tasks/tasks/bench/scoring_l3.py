"""Relative rotation and tool/contact witnesses, independent of commands or notes."""

import math

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released, table_placed, upright


def bottle(e, rows):
    body, cap = rows["bottle"], rows["cap"]
    p = e.spec.parameters
    t = relative(cap, body)
    angle = math.atan2(t[1, 0], t[0, 0])
    aligned = np.linalg.norm(t[:2, 3]) < 0.006 and t[2, 2] > 0.96
    supported = body["bench_force_n"] > 0.05 and upright(body, 0.3)
    if e.previous_angle is not None:
        delta = (angle - e.previous_angle + math.pi) % (2 * math.pi) - math.pi
        # Accumulate signed relative turning while both objects are physically held.
        # Large sampling discontinuities cannot manufacture thread progress.
        if (
            aligned
            and supported
            and t[2, 3] <= p["engaged_cap_z_max"]
            and held(body, "left")
            and held(cap, "right")
            and abs(delta) < math.pi / 2
        ):
            e.net_turn += delta
    e.previous_angle = angle
    rise = float(t[2, 3] - p["initial_cap_z"])
    gap = float(t[2, 3] - p["cap_half"] - p["neck_top"])
    if (
        e.net_turn >= p["minimum_turn"]
        and rise >= p["rise"]
        and gap > 0.004
        and cap["pair_forces"].get("bottle", 0) < 0.05
        and held(body, "left")
        and held(cap, "right")
        and supported
    ):
        e.flags.add("threaded_disengagement")
    separate = np.linalg.norm(np.array(body["pose"][:2]) - cap["pose"][:2]) > 0.05
    complete = (
        "threaded_disengagement" in e.flags
        and separate
        and table_placed(body)
        and table_placed(cap)
        and upright(body, 0.3)
    )
    return (
        complete,
        {
            "net_relative_turn_rad": float(e.net_turn),
            "turn_count_scope": "both parts held, bottle upright on table, cap within thread engagement height",
            "bottle_supported_on_table": bool(supported),
            "cap_rise_m": rise,
            "axial_clearance_m": gap,
            "threaded_disengagement": "threaded_disengagement" in e.flags,
        },
        "",
    )


def pushing(e, rows):
    from .push_t import footprint_outside_area

    block = rows["t_block"]
    article = e.spec.design.articles[0]
    p = e.spec.parameters
    t = pose_transform(block["pose"])
    relative_pose = np.linalg.inv(pose_transform(article.goal)) @ t
    yaw = math.atan2(relative_pose[1, 0], relative_pose[0, 0])
    xy = np.linalg.norm(t[:2, 3] - article.goal[:2])
    outside_area = footprint_outside_area(article, block["pose"], p["target_rectangles"])
    contact = max(block["fingers"]["left"]) > 0.05
    supported = block["bench_force_n"] > 0.05
    # A push needs measured motion during sustained left-finger contact on the
    # table. Touching an already placed block is not a task completion witness.
    if contact and supported:
        if e.push_contact_pose is None:
            e.push_contact_pose = (t[0, 3], t[1, 3], yaw)
        start = e.push_contact_pose
        translation = np.linalg.norm(t[:2, 3] - start[:2])
        rotation = abs((yaw - start[2] + math.pi) % (2 * math.pi) - math.pi)
        if translation >= p["push_translation_m"] or rotation >= p["push_rotation_rad"]:
            e.flags.add("gripper_pushed_block")
    else:
        e.push_contact_pose = None
    failure = ""
    if max(block["fingers"]["right"]) > 0.05:
        e.flags.add("observation_arm_touched_block")
    if min(c[2] for c in block["corners"]) > p["max_lift_m"]:
        e.flags.add("block_lifted_from_table")
    if not upright(block, 0.30):
        e.timers.setdefault("t_block_tipped", e.last_time)
        if e.last_time - e.timers["t_block_tipped"] >= 0.15 - 1e-9:
            e.flags.add("block_toppled")
    else:
        e.timers.pop("t_block_tipped", None)
    if "observation_arm_touched_block" in e.flags:
        failure = "The observation arm touched the T block"
    if "block_lifted_from_table" in e.flags or "block_toppled" in e.flags:
        failure = "T block was lifted or toppled; use planar pushing on the table"
    complete = (
        outside_area <= 1e-9
        and abs(yaw) <= p["yaw_tolerance_rad"]
        and upright(block, 0.18)
        and quiet(block)
        and supported
        and released(block)
        and "gripper_pushed_block" in e.flags
    )
    return (
        complete,
        {
            "position_error_m": float(xy),
            "yaw_error_rad": abs(yaw),
            "target_padding_m": p["target_padding_m"],
            "footprint_outside_area_m2": outside_area,
            "inside_target_outline": outside_area <= 1e-9,
            "yaw_tolerance_rad": p["yaw_tolerance_rad"],
            "gripper_push_witness": "gripper_pushed_block" in e.flags,
            "grippers_clear": released(block),
        },
        failure,
    )


def sweeping(e, rows):
    pan, broom = rows["dustpan"], rows["broom"]
    count = e.spec.parameters["count"]
    holds = held(pan, "right") and held(broom, "left")
    inside = []
    failure = ""
    for i in range(count):
        name = f"block_{i}"
        block = rows[name]
        t = relative(block, pan)
        contained = bool(
            abs(t[0, 3]) < 0.060
            and -0.042 < t[1, 3] < 0.045
            and 0.004 < t[2, 3] < 0.032
        )
        support = block["pair_forces"].get("dustpan", 0) > 0.015
        inside.append(contained and support)
        if broom["pair_forces"].get(name, 0) > 0.015 and holds:
            e.flags.add("swept_" + name)
        if any(max(fs) > 0.05 for fs in block["fingers"].values()):
            failure = "A block was manipulated directly by a gripper"
    complete = (
        sum(inside) >= e.spec.parameters["required"]
        and holds
        and upright(pan, 0.20)
        and any(f.startswith("swept_") for f in e.flags)
        and quiet(pan)
        and all(quiet(rows[f"block_{i}"]) for i in range(count))
    )
    return (
        complete,
        {
            "blocks_in_pan": sum(inside),
            "required": e.spec.parameters["required"],
            "both_tools_held": holds,
        },
        failure,
    )


def push_drawer(e, rows):
    upper, lower = rows["upper_drawer"], rows["lower_drawer"]
    p = e.spec.parameters
    outward = np.asarray(p["outward_axis_world"])
    displacement = np.asarray(upper["pose"][:3]) - e.initial["upper_drawer"][:3]
    travel = float(displacement @ outward)
    phase = upper.get("push_latch", {}).get("phase", "unknown")
    if travel <= -p["press_m"] and max(upper["fingers"]["left"]) > 0.05:
        e.flags.add("upper_drawer_pressed")
    if ("upper_drawer_pressed" in e.flags and phase == "open" and released(upper)
            and p["pop_min_m"] <= travel < p["open_min_m"]):
        e.flags.add("upper_drawer_popped_free")
    if ("upper_drawer_popped_free" in e.flags and held(upper, "left")
            and travel >= p["open_min_m"]):
        e.flags.add("upper_drawer_pulled_open")
    lower_drift = np.linalg.norm(np.asarray(lower["pose"][:3]) - e.initial["lower_drawer"][:3])
    alignment = np.linalg.norm(displacement - travel * outward)
    failure = ""
    lower_contact = any(max(fs) > 0.5 for fs in lower["fingers"].values())
    if lower_contact:
        e.timers.setdefault("wrong_layer_contact", e.last_time)
        if e.last_time - e.timers["wrong_layer_contact"] >= 0.15 - 1e-9:
            failure = "The lower drawer was operated; the target is the upper drawer"
    else:
        e.timers.pop("wrong_layer_contact", None)
    if lower_drift > 0.003 or alignment > 0.004 or not upright(upper, 0.10):
        failure = "Cabinet/drawer guide constraint was violated"
    if travel > p["outward_stop_m"] + 0.003 or travel < p["inward_stop_m"] - 0.003:
        failure = "Upper drawer exceeded its mechanical stop"
    if np.linalg.norm(upper["linear_velocity"]) > p["max_speed_m_s"]:
        failure = "Upper drawer motion exceeded the declared gentle-motion limit"
    if e.peak_force > p["max_finger_force_n"]:
        failure = "Finger contact exceeded the engineering force threshold"
    complete = ("upper_drawer_pulled_open" in e.flags and phase == "open"
                and travel >= p["open_min_m"] and released(upper) and quiet(upper))
    next_stage = next((stage for flag, stage in (
        ("upper_drawer_pressed", "press_upper_drawer"),
        ("upper_drawer_popped_free", "release_and_observe_popout"),
        ("upper_drawer_pulled_open", "grasp_and_pull_upper_drawer"),
    ) if flag not in e.flags), "release_and_stabilize")
    return complete, {
        "target_layer": "upper", "travel_m": float(travel), "latch_phase": phase,
        "next_stage": next_stage,
        "pressed_by_manipulation_arm": "upper_drawer_pressed" in e.flags,
        "released_popout_observed": "upper_drawer_popped_free" in e.flags,
        "grasped_pull_observed": "upper_drawer_pulled_open" in e.flags,
        "released_at_finish": released(upper), "lower_drawer_drift_m": float(lower_drift),
    }, failure


RULES = {"L3-03": bottle, "L3-14": pushing, "L3-15": sweeping, "L3-16": push_drawer}
