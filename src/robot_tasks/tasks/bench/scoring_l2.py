"""Process-aware L2 scoring; force/strain limits are declared simulation proxies."""

import itertools
import math

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released, table_placed, upright


def books(e, rows):
    middle, top, base = (rows[n] for n in ("middle_book", "upper_book", "lower_book"))
    extracted = (
        abs(middle["pose"][0] - base["pose"][0])
        >= e.spec.parameters["extract_distance"]
    )
    if extracted and held(top) and held(middle):
        e.flags.add("supported_extraction")
    failure = ""
    if "supported_extraction" not in e.flags and (
        top["bench_force_n"] > 0.05 or not upright(top, 0.5)
    ):
        failure = "Upper book lost support before middle-book extraction"
    complete = (
        "supported_extraction" in e.flags
        and table_placed(middle)
        and table_placed(top)
        and upright(base, 0.3)
        and quiet(base)
    )
    return (
        complete,
        {
            "middle_extracted": bool(extracted),
            "support_witness": "supported_extraction" in e.flags,
        },
        failure,
    )


def bowl(e, rows):
    bowl = rows["bowl"]
    p = e.spec.parameters
    if rows["feed_gate"]["pose"][0] > e.initial["feed_gate"][0] + 0.01:
        e.flags.add("feeding_started")
    inside = []
    spilled = []
    for i in range(p["count"]):
        r = rows[f"load_{i}"]
        t = relative(r, bowl)
        radius = np.linalg.norm(t[:2, 3])
        # Entire 16-mm block fits below the rim and above the bowl floor.
        contained = (
            radius + 0.012 < 0.045 + max(0, t[2, 3]) * (0.019 / 0.055)
            and 0.010 <= t[2, 3] <= 0.048
        )
        inside.append(bool(contained))
        if "feeding_started" in e.flags and (
            r["bench_force_n"] > 0.05 or t[2, 3] < -0.015
        ):
            spilled.append(i)
    holding = (
        held(bowl, "left") and held(bowl, "right") and bowl["bench_force_n"] < 0.05
    )
    steady_load = all(inside) and holding and upright(bowl, 0.18) and quiet(bowl)
    if steady_load:
        e.timers.setdefault("loaded_hold", e.last_time)
        if e.last_time - e.timers["loaded_hold"] >= 2.0:
            e.flags.add("loaded_hold_completed")
    else:
        e.timers.pop("loaded_hold", None)
    complete = (
        all(inside)
        and "loaded_hold_completed" in e.flags
        and table_placed(bowl)
        and upright(bowl, 0.18)
    )
    return (
        complete,
        {
            "contained_blocks": sum(inside),
            "required_blocks": p["count"],
            "holding_both_handles": holding,
            "loaded_hold_completed": "loaded_hold_completed" in e.flags,
            "spilled": spilled,
        },
        ("Load spilled" if spilled else ""),
    )


def latch(e, rows):
    lid, button, eraser = (rows[n] for n in ("lid", "latch", "eraser"))
    rotation = pose_transform(lid["pose"])[:3, :3]
    angle = math.atan2(rotation[2, 0], rotation[0, 0])
    extension = float(relative(button, lid)[0, 3] - 0.096)
    if extension > 0.01 and held(button):
        e.flags.add("latch_retracted")
    if angle > 0.65 and "latch_retracted" in e.flags:
        e.flags.add("box_opened")
    outside = np.linalg.norm(np.array(eraser["pose"][:2]) - (0.37, 0)) > 0.13
    if outside and "box_opened" in e.flags and "eraser" in e.grasped:
        e.flags.add("item_removed")
    closed = abs(angle) < 0.06 and abs(extension) < 0.004
    complete = (
        "item_removed" in e.flags
        and outside
        and table_placed(eraser)
        and closed
        and released(lid)
        and released(button)
        and quiet(lid)
        and quiet(button)
    )
    return (
        complete,
        {
            "lid_open_angle_rad": angle,
            "latch_extension_m": extension,
            "closed_and_latched_geometry": closed,
        },
        "",
    )


def jelly(e, rows):
    p = e.spec.parameters
    points = np.array([rows[f"jelly_{i}"]["pose"][:3] for i in range(8)])
    rest = np.array(p["rest_points"])
    strain = max(
        abs(
            np.linalg.norm(points[i] - points[j]) / np.linalg.norm(rest[i] - rest[j])
            - 1
        )
        for i, j in itertools.combinations(range(8), 2)
    )
    e.max_strain = max(e.max_strain, float(strain))
    centroid = points.mean(axis=0)
    # Fingers can contact different elastic cells; the aggregate, not a single cell, is grasped.
    forces = np.sum([r["fingers"]["left"] for r in rows.values()], axis=0)
    if min(forces) > 0.05 and centroid[2] > rest[:, 2].mean() + 0.015:
        e.flags.add("elastic_object_lifted")
    contained = bool(
        np.all(np.abs(points[:, :2] - p["target_xy"]) + 0.011 < p["target_half"])
    )
    supported = sum(r["bench_force_n"] for r in rows.values()) > 0.1
    complete = (
        "elastic_object_lifted" in e.flags
        and contained
        and supported
        and all(released(r) and quiet(r) for r in rows.values())
    )
    return (
        complete,
        {
            "strain": float(strain),
            "peak_strain": e.max_strain,
            "within_target": contained,
            "damage_model": "elastic strain proxy only",
        },
        (
            "Elastic proxy exceeded strain limit"
            if e.max_strain > p["strain_limit"]
            else ""
        ),
    )


def drawer(e, rows):
    r = rows["drawer"]
    travel = r["pose"][0] - e.initial["drawer"][0]
    if travel >= 0.10 and held(r):
        e.flags.add("opened_while_held")
    at_stop = travel < 0.006 or travel > 0.124
    impact = at_stop and np.linalg.norm(r["linear_velocity"]) > 0.25
    complete = (
        "opened_while_held" in e.flags
        and abs(travel) < 0.006
        and released(r)
        and quiet(r)
    )
    return (
        complete,
        {
            "opening_m": float(travel),
            "opened_then_closed": "opened_while_held" in e.flags,
            "impact_speed_limit_m_s": 0.25,
        },
        ("Excessive speed at drawer stop" if impact else ""),
    )


RULES = {"L2-03": books, "L2-04": bowl, "L2-05": latch, "L2-06": jelly, "L2-07": drawer}
