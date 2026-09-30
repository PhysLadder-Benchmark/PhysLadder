"""Relative rotation and tool/contact witnesses, independent of commands or notes."""

import math

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, table_placed, upright


def bottle(e, rows):
    body, cap = rows["bottle"], rows["cap"]
    p = e.spec.parameters
    t = relative(cap, body)
    angle = math.atan2(t[1, 0], t[0, 0])
    aligned = np.linalg.norm(t[:2, 3]) < 0.006 and t[2, 2] > 0.96
    if e.previous_angle is not None:
        delta = (angle - e.previous_angle + math.pi) % (2 * math.pi) - math.pi
        # Accumulate signed relative turning while both objects are physically held.
        # Large sampling discontinuities cannot manufacture thread progress.
        if (
            aligned
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
            "turn_count_scope": "only while the cap is within the thread engagement height",
            "cap_rise_m": rise,
            "axial_clearance_m": gap,
            "threaded_disengagement": "threaded_disengagement" in e.flags,
        },
        "",
    )


def pushing(e, rows):
    block, tool = rows["t_block"], rows["tool"]
    article = e.spec.design.articles[0]
    t = pose_transform(block["pose"])
    yaw = math.atan2(t[1, 0], t[0, 0])
    xy = np.linalg.norm(t[:2, 3] - article.goal[:2])
    contact = tool["pair_forces"].get("t_block", 0)
    if contact > 0.05 and held(tool) and "tool" in e.grasped:
        e.flags.add("tool_pushed_block")
    direct = any(max(fs) > 0.05 for fs in block["fingers"].values())
    if direct:
        e.flags.add("direct_block_contact")
    complete = (
        xy <= 0.012
        and abs(yaw) <= math.radians(12)
        and upright(block, 0.18)
        and quiet(block)
        and block["bench_force_n"] > 0.05
        and held(tool)
        and contact < 0.05
        and "tool_pushed_block" in e.flags
        and "direct_block_contact" not in e.flags
    )
    return (
        complete,
        {
            "position_error_m": float(xy),
            "yaw_error_rad": abs(yaw),
            "tool_contact_witness": "tool_pushed_block" in e.flags,
        },
        ("T block was touched directly by a gripper" if direct else ""),
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


RULES = {"L3-03": bottle, "L3-14": pushing, "L3-15": sweeping}
