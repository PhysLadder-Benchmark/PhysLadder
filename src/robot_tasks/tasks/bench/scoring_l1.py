"""Containment, release and load-bearing criteria for the active L1 tasks."""

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released, upright


def marker(e, rows):
    r = rows["marker"]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    tip = t[:3, 3]
    axis = t[:3, 2]
    depth = p["rim"] - tip[2]
    rim = tip + axis * (depth / max(axis[2], 1e-6))
    inside = (
        upright(r)
        and tip[2] >= p["floor"] - 0.002
        and depth >= p["required_depth"]
        and max(
            np.linalg.norm(tip[:2] - p["holder_xy"]),
            np.linalg.norm(rim[:2] - p["holder_xy"]),
        )
        + 0.016
        <= p["radius"]
    )
    support = r["fixture_force_n"] > 0.2 and r["bench_force_n"] < 0.05
    done = inside and support and "marker" in e.grasped and released(r) and quiet(r)
    return (
        done,
        {
            "inside_holder": bool(inside),
            "depth_m": float(depth),
            "fixture_supported": bool(support),
        },
        "",
    )


def insertion(e, rows):
    name = "plug" if e.spec.design.task_id == "L1-03" else "key"
    r = rows[name]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    axis = t[:3, 2]
    depth = p["rim"] - t[2, 3]
    rim = t[:3, 3] + axis * (depth / max(axis[2], 1e-6))
    # Cross-section projected into the rectangular keyway; 180-degree yaw symmetry is natural.
    extent = np.abs(t[:2, :2]) @ (np.array(p["tongue"]) / 2)
    centres = np.vstack((t[:2, 3], rim[:2])) - p["socket_xy"]
    inside = bool(
        upright(r, 0.18)
        and depth >= p["depth"]
        and t[2, 3] >= p["floor"] - 0.002
        and np.all(np.abs(centres) + extent <= np.array(p["opening"]) / 2 + 0.0005)
    )
    release_ok = name == "key" or released(r)
    support = held(r) or r["fixture_force_n"] > 0.05
    return (
        inside and name in e.grasped and release_ok and support and quiet(r),
        {
            "inside_socket": inside,
            "depth_m": float(depth),
            "release_required": name == "plug",
        },
        "",
    )


def mug(e, rows):
    r = rows["mug"]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    a = (np.array(p["hook_start"]) - t[:3, 3]) @ t[:3, :3]
    b = (np.array(p["hook_end"]) - t[:3, 3]) @ t[:3, :3]
    through = False
    if abs(b[1] - a[1]) > 1e-8:
        f = -a[1] / (b[1] - a[1])
        hit = a + f * (b - a)
        offset = hit - np.array(p["handle_centre"])
        through = bool(
            0 < f < 1
            # Resting contact at the inner handle wall is valid. Include the
            # declared millimetre contact tolerance instead of rejecting tangency.
            and abs(offset[0]) + 0.004 <= p["handle_half"][0] + 0.001
            and abs(offset[2]) <= p["handle_half"][1] + 0.0045
        )
    supported = (
        r["fixture_force_n"] > 0.25
        and r["bench_force_n"] < 0.05
        and np.min(np.asarray(r["corners"])[:, 2]) > 0.03
    )
    return (
        through and supported and "mug" in e.grasped and released(r) and quiet(r),
        {"hook_through_handle": through, "hanging_support": bool(supported)},
        "",
    )


def cups(e, rows):
    top, base = rows["upper_cup"], rows["lower_cup"]
    t = relative(top, base)
    contained = bool(
        np.linalg.norm(t[:2, 3]) < 0.005
        and 0.002 <= t[2, 3] <= 0.013
        and upright(top)
        and upright(base)
    )
    supported = (
        top["pair_forces"].get("lower_cup", 0) > 0.1 and base["bench_force_n"] > 0.1
    )
    return (
        contained
        and supported
        and "upper_cup" in e.grasped
        and released(top)
        and released(base)
        and quiet(top)
        and quiet(base),
        {"nested": contained, "cup_on_cup_support": bool(supported)},
        "",
    )


def flower(e, rows):
    tip = rows["stem_0"]
    p = e.spec.parameters
    t = pose_transform(tip["pose"])
    end = t[:3, 3] - t[:3, 2] * 0.014
    inside = bool(
        np.linalg.norm(end[:2] - p["vase_xy"]) + 0.008 < p["radius"]
        and 0.002 <= end[2] <= p["rim"] - p["depth"]
    )
    supported = sum(r["fixture_force_n"] for r in rows.values()) > 0.1
    released_all = all(released(r) for r in rows.values())
    # A soft flower may naturally drape below the rim. Require the whole
    # flower to be carried by the vase, rather than resting its head on the table.
    head_clear = (
        np.min(np.asarray(rows["flower_head"]["corners"])[:, 2]) > 0.025
        and sum(r["bench_force_n"] for r in rows.values()) < 0.05
    )
    return (
        inside
        and supported
        and head_clear
        and bool(e.grasped)
        and released_all
        and all(quiet(r) for r in rows.values()),
        {
            "stem_end_inside": inside,
            "flower_clear_of_table": bool(head_clear),
            "proxy_only": True,
        },
        "",
    )


RULES = {
    "L1-01": marker,
    "L1-03": insertion,
    "L1-05": insertion,
    "L1-07": mug,
    "L1-08": cups,
    "L1-09": flower,
}
