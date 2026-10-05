"""Containment, release and load-bearing criteria for the active L1 tasks."""

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released, upright


def marker(e, rows):
    r = rows["marker"]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    ends = np.array([t[:3, 3], t[:3, 3] + t[:3, 2] * p["marker_height"]])
    tip, upper = ends[np.argsort(ends[:, 2])]
    axis = (upper - tip) / p["marker_height"]
    depth = p["rim"] - tip[2]
    tilt_radius_z = p["marker_radius"] * np.linalg.norm(axis[:2])
    tip_radius = p["radius"] + (p["mouth_radius"] - p["radius"]) * np.clip(
        (tip[2] - p["floor"]) / (p["rim"] - p["floor"]), 0, 1)
    if upper[2] > p["rim"]:
        rim = tip + axis * (depth / max(axis[2], 1e-6))
        rim_extent = p["marker_radius"] / max(axis[2], 1e-6)
    else:
        # A fully contained object need not intersect the rim at all.
        rim, rim_extent = upper, p["marker_radius"]
    # Cup walls are separate convex sectors. Use their inscribed, rather than
    # vertex, radius so the conical containment test cannot accept penetration.
    polygon_factor = np.cos(np.pi / 48)
    inside = (
        tip[2] - tilt_radius_z >= p["floor"] - 0.001
        and depth > tilt_radius_z + 0.001  # An end is inside, not merely resting on the rim.
        and np.linalg.norm(tip[:2] - p["holder_xy"]) + p["marker_radius"] <= tip_radius * polygon_factor + 0.001
        and np.linalg.norm(rim[:2] - p["holder_xy"]) + rim_extent <= p["mouth_radius"] * polygon_factor + 0.001
    )
    support = r["fixture_force_n"] > p["min_support_n"] and r["bench_force_n"] < 0.05
    # Initial upright tabletop support is expected. Only a later landing or
    # the pen's body lying on the table ends the episode; tilt in free space is allowed.
    table_z = p["table_z"]
    if tip[2] - tilt_radius_z > table_z + 0.005 and r["bench_force_n"] < 0.05:
        e.flags.add("marker_left_table")
    body_on_table = ends[:, 2].mean() - tilt_radius_z <= table_z + 0.003
    if (not held(r) and r["bench_force_n"] > 0.05
            and ("marker_left_table" in e.flags or body_on_table)):
        e.flags.add("marker_dropped_on_table")
    failure = "Marker dropped onto the tabletop" if "marker_dropped_on_table" in e.flags else ""
    # Completion is an observed outcome, not a prescribed grip or release trajectory.
    done = inside and support and released(r) and quiet(r)
    return (
        done,
        {
            "inside_holder": bool(inside),
            "depth_m": float(depth),
            "fixture_supported": bool(support),
            "dropped_on_table": bool(failure),
        },
        failure,
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
    # Section in the horizontal socket plane; account for tilt geometrically
    # rather than imposing a separate angle limit.
    section = t[:2, :2] - np.outer(axis[:2], t[2, :2]) / max(axis[2], 1e-6)
    extent = np.abs(section) @ (np.array(p["tongue"]) / 2)
    if "slot_centres" in p:
        # Match both native blades to both native holes, including 180-degree
        # symmetry. A body-centre-only test would accept a pin outside its slot.
        slots = np.array(p["slot_centres"])
        tips = np.array([t[:2, 3] + t[:2, :2] @ c for c in slots])
        at_rim = tips + axis[:2] * depth / max(axis[2], 1e-6)
        centres = []
        for order in (slots, slots[::-1]):
            offset = np.vstack((tips - order - p["socket_xy"], at_rim - order - p["socket_xy"]))
            centres.append(offset)
        cross_section_fits = any(np.all(np.abs(c) + extent <= np.array(p["opening"]) / 2 + 0.00015)
                                 for c in centres)
    else:
        centres = np.vstack((t[:2, 3], rim[:2])) - p["socket_xy"]
        cross_section_fits = np.all(np.abs(centres) + extent <= np.array(p["opening"]) / 2 + 0.0005)
    inside = bool(
        axis[2] > 1e-6
        and depth >= p["depth"]
        and t[2, 3] >= p["floor"] - 0.002
        and cross_section_fits
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
        direction = (b - a) / np.linalg.norm(b - a)
        radius = p.get("hook_radius", 0.004)
        # A sloping round peg makes an ellipse in the handle's XZ plane.
        radial_x = radius * np.sqrt(1 + (direction[0] / direction[1]) ** 2)
        radial_z = radius * np.sqrt(1 + (direction[2] / direction[1]) ** 2)
        through = bool(
            0 < f < 1
            # Resting contact at the inner handle wall is valid. Include the
            # declared millimetre contact tolerance instead of rejecting tangency.
            and abs(offset[0]) + radial_x <= p["handle_half"][0] + 0.001
            and abs(offset[2]) + radial_z <= p["handle_half"][1] + 0.001
        )
    supported = (
        r["fixture_force_n"] > 0.25
        and r["bench_force_n"] < 0.05
        and np.min(np.asarray(r["corners"])[:, 2]) > p.get("table_z", 0) + 0.03
    )
    return (
        through and supported and "mug" in e.grasped and released(r),
        {"hook_through_handle": through, "hanging_support": bool(supported),
         "release_required": True, "stability_wait_s": 0.0},
        "",
    )


def cups(e, rows):
    top, base = rows["upper_cup"], rows["lower_cup"]
    p = e.spec.parameters
    t = relative(top, base)
    contained = _cup_contained(t, p) and pose_transform(base["pose"])[2, 2] > 1e-6
    supported = (
        top["pair_forces"].get("lower_cup", 0) > p["min_pair_support_n"]
        and base["bench_force_n"] > p["min_bench_support_n"]
    )
    return (
        contained
        and supported
        and "upper_cup" in e.grasped
        and released(top)
        and released(base)
        and quiet(top)
        and quiet(base),
        {"nested": contained, "cup_on_cup_support": bool(supported),
         "relative_bottom_height_m": float(t[2, 3])},
        "",
    )


def _cup_contained(transform, p):
    """Clip the upper cup's outer frustum at the lower rim and test cavity fit.

    Uses the native wall collider dimensions with declared contact tolerance.
    Requires a full bottom ring inside; rim balancing and wall/floor penetration
    cannot satisfy this check. Offset and tilt follow geometry, not ideal poses.
    """
    count = 48
    angles = np.arange(count) * (2 * np.pi / count)
    local = np.array([[(radius * np.cos(a), radius * np.sin(a), z) for a in angles]
                      for radius, z in ((p["cup_outer_bottom_radius"], 0.0),
                                        (p["cup_outer_top_radius"], p["cup_height"]))])
    rings = local @ transform[:3, :3].T + transform[:3, 3]
    rim, floor = p["cup_height"], p["cup_floor"]
    tol = p["cup_contact_tolerance"]
    if transform[2, 2] <= 1e-6 or rings[0, :, 2].max() >= rim - tol:
        return False
    vertices = rings.reshape(-1, 3)
    starts = np.concatenate((rings[0], rings[1], rings[0]))
    ends = np.concatenate((np.roll(rings[0], -1, axis=0),
                           np.roll(rings[1], -1, axis=0), rings[1]))
    crosses = (starts[:, 2] < rim) != (ends[:, 2] < rim)
    a, b = starts[crosses], ends[crosses]
    intersections = a + (b - a) * ((rim - a[:, 2]) / (b[:, 2] - a[:, 2]))[:, None]
    clipped = np.concatenate((vertices[vertices[:, 2] <= rim], intersections))
    radius = p["cup_inner_bottom_radius"] + (
        p["cup_inner_top_radius"] - p["cup_inner_bottom_radius"]) * np.clip(
            (clipped[:, 2] - floor) / (rim - floor), 0, 1)
    return bool(np.all(clipped[:, 2] >= floor - tol)
                and np.all(np.linalg.norm(clipped[:, :2], axis=1)
                           <= radius * np.cos(np.pi / count) + tol))


def flower(e, rows):
    r = rows["rose"]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    tip, axis = t[:3, 3], t[:3, 2]
    depth = p["rim"] - tip[2]
    rim_distance = depth / max(axis[2], 1e-6)
    at_rim = tip + axis * rim_distance
    tip_radius = p["radius"] + (p["mouth_radius"] - p["radius"]) * np.clip(
        (tip[2] - p["floor"]) / (p["rim"] - p["floor"]), 0, 1)
    cross_section = p["stem_radius"] / max(axis[2], 1e-6)
    polygon_factor = np.cos(np.pi / 48)
    inside = bool(
        axis[2] > 1e-6
        and tip[2] >= p["floor"] - 0.001
        and depth > p["stem_radius"] * np.linalg.norm(axis[:2]) + 0.001
        and 0 < rim_distance < p["stem_length"]
        and np.linalg.norm(tip[:2] - p["vase_xy"]) + p["stem_radius"] <= tip_radius * polygon_factor + 0.001
        and np.linalg.norm(at_rim[:2] - p["vase_xy"]) + cross_section <= p["mouth_radius"] * polygon_factor + 0.001
    )
    # Allow a naturally leaning stem, but the wider bud must stay above the rim.
    head_low = tip[2] + axis[2] * p["head_start"] - np.linalg.norm(axis[:2]) * p["head_radius"]
    head_clear = bool(head_low > p["rim"] and r["bench_force_n"] < 0.01)
    supported = bool(r["fixture_force_n"] > p["min_support_n"])
    return (
        inside and head_clear and supported and "rose" in e.grasped and released(r) and quiet(r),
        {"stem_end_inside": inside, "depth_m": float(depth),
         "flower_clear_of_table": head_clear, "head_above_rim": head_clear,
         "fixture_supported": supported, "rigid_flower": True},
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
