"""Process-aware L2 scoring; force/strain limits are declared simulation proxies."""

import itertools
import math

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released, sustained, table_placed, upright


def books(e, rows):
    middle = rows["middle_book"]
    p = e.spec.parameters
    outward = np.asarray(p["outward_axis_world"])
    travel = float((np.asarray(middle["pose"][:3]) - e.initial["middle_book"][:3]) @ outward)
    # The trailing corner must cross the fixed stack's exit plane. Projection
    # includes every corner, so translating or rotating sideways cannot fake it.
    rear_clearance = float(np.min((np.asarray(middle["corners"]) - p["stack_front_world"]) @ outward))
    extracted = rear_clearance >= p["clearance_m"] and travel >= p["minimum_travel_m"]
    grasped = held(middle, "left")
    if extracted and grasped:
        e.flags.add("middle_extracted_while_held")
    complete = extracted and grasped and quiet(middle)
    return complete, {"middle_extracted": bool(extracted), "outward_travel_m": travel,
                      "rear_clearance_m": rear_clearance, "outward_axis_world": outward.tolist(),
                      "held_by_left": grasped,
                      "adjacent_books": "fixed fixtures; no support evaluation",
                      "release_required": False}, ""


def ball_contained(position, p):
    """Sphere fits within every sloped polygon wall, floor and rim of the bowl."""
    x, y, z = position
    r = p["ball_radius_m"]
    n = p["wall_segments"]
    cosine = math.cos(math.pi / n)
    slope = (p["top_radius_m"] - p["bottom_radius_m"]) / (p["height_m"] - p["floor_m"])
    boundary = (p["bottom_radius_m"] + slope * (z - p["floor_m"])) * cosine
    margin = r * math.sqrt(1 + (slope * cosine) ** 2)
    normals = np.arange(n) * (2 * math.pi / n) + math.pi / n
    return bool(z - r >= p["floor_m"] - 0.0015 and z + r <= p["height_m"] + 0.001
                and np.all(x * np.cos(normals) + y * np.sin(normals) + margin <= boundary + 0.001))


def bowl(e, rows):
    bowl = rows["bowl"]
    p = e.spec.parameters
    inside = []
    for i in range(p["count"]):
        row = rows[f"load_{i}"]
        point = relative(row, bowl)[:3, 3]
        inside.append(ball_contained(point, p))
        # A previously spilled ball remains a spill even if returned to the bowl.
        if (row["bench_force_n"] > 0.005 or point[2] < -p["ball_radius_m"]
                or np.linalg.norm(point[:2]) - p["ball_radius_m"] > p["top_radius_m"] + 0.008):
            e.flags.add(f"spilled_ball_{i}")
    spilled = [i for i in range(p["count"]) if f"spilled_ball_{i}" in e.flags]
    holding = held(bowl, "left") and held(bowl, "right")
    if holding and bowl["bench_force_n"] < 0.05 and bowl["pose"][2] - e.initial["bowl"][2] >= p["lift_m"]:
        e.flags.add("loaded_bowl_lifted")
    target_error = float(np.linalg.norm(np.asarray(bowl["pose"][:2]) - p["target_xy"]))
    transported = ("loaded_bowl_lifted" in e.flags and table_placed(bowl)
                   and upright(bowl, 0.18) and target_error <= p["target_tolerance_m"])
    complete = transported and (all(inside) or bool(spilled))
    return complete, {"contained_balls": sum(inside), "required_balls": p["count"],
                      "holding_both_handles": holding, "target_error_m": target_error,
                      "loaded_bowl_lifted": "loaded_bowl_lifted" in e.flags, "spilled": spilled,
                      "transport_complete": bool(transported), "no_spill": not bool(spilled),
                      "constraint_violations": ["Load spilled"] if spilled else []}, ""


def latch(e, rows):
    lid, button, eraser = (rows[n] for n in ("lid", "latch", "eraser"))
    rotation = (np.linalg.inv(pose_transform(e.initial["lid"])) @ pose_transform(lid["pose"]))[:3, :3]
    angle = math.atan2(rotation[2, 0], rotation[0, 0])
    extension = float(relative(button, lid)[0, 3] - e.spec.parameters["latch_closed_x_local"])
    if extension > 0.01 and held(button, "left"):
        e.flags.add("latch_retracted")
    if angle > 0.65 and "latch_retracted" in e.flags:
        e.flags.add("box_opened")
    outside = np.linalg.norm(np.array(eraser["pose"][:2]) - e.spec.parameters["box_xy"]) > 0.13
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
            "transport_complete": bool(complete),
            "strain_limit": p["strain_limit"],
            "strain_limit_satisfied": e.max_strain <= p["strain_limit"],
            "constraint_violations": (["Elastic proxy exceeded strain limit"]
                                      if e.max_strain > p["strain_limit"] else []),
            "damage_model": "elastic strain proxy only",
        },
        "",
    )


def drawer(e, rows):
    row = rows["drawer"]
    p = e.spec.parameters
    displacement = np.asarray(row["pose"][:3]) - e.initial["drawer"][:3]
    outward = np.asarray(p["outward_axis_world"])
    travel = float(displacement @ outward)
    guide_deviation = np.linalg.norm(displacement - travel * outward) > 0.008 or not upright(row, 0.15)
    beyond_stop = travel < -0.006 or travel > p["max_travel_m"] + 0.006
    failure = ""
    if sustained(e, "drawer_guide_deviation", guide_deviation):
        failure = "Drawer guide constraint was persistently violated"
    if sustained(e, "drawer_beyond_stop", beyond_stop):
        failure = "Drawer persistently exceeded its mechanical stop"
    if travel >= p["required_open_m"] and held(row, "left"):
        e.flags.add("opened_while_held")
    at_stop = travel < 0.006 or travel > p["max_travel_m"] - 0.006
    impact = at_stop and np.linalg.norm(row["linear_velocity"]) > p["max_speed_m_s"]
    if impact:
        e.flags.add("drawer_speed_reference_exceeded")
    complete = ("opened_while_held" in e.flags and p["required_open_m"] <= travel <= p["max_travel_m"] + 0.003
                and released(row) and quiet(row) and not guide_deviation and not beyond_stop)
    return complete, {"opening_m": travel, "opened_while_held": "opened_while_held" in e.flags,
                      "final_state": "released and open", "impact_speed_reference_m_s": p["max_speed_m_s"],
                      "peak_speed_m_s": e.peak_speeds.get("drawer", 0.0),
                      "quality_warnings": (["Drawer speed at stop exceeded engineering reference"]
                                           if "drawer_speed_reference_exceeded" in e.flags else [])}, failure


RULES = {"L2-03": books, "L2-04": bowl, "L2-05": latch, "L2-06": jelly, "L2-07": drawer}
