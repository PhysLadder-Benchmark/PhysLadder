"""Direct gripper pushing with a generous, non-colliding T target outline."""

from dataclasses import replace
import math

import numpy as np

from ...robots.transforms import pose_transform
from ..l1.specs import RED, Article, box, pose
from .geometry import design
from .spec import BenchSpec


def _rectangle(part, padding=0.0):
    x, y = part.position[:2]
    hx, hy = np.asarray(part.size[:2]) / 2 + padding
    return (float(x - hx), float(x + hx), float(y - hy), float(y + hy))


def build():
    parts = (
        box("crossbar", (0.10, 0.030, 0.026), (0, 0.025, 0), RED),
        box("stem", (0.030, 0.070, 0.026), (0, -0.025, 0), RED),
    )
    article = Article("t_block", parts, pose(0.39, 0.43, 0.014, yaw=0.65),
                      pose(0.39, 0.22, 0.013), 0.10)
    padding = 0.020
    cross, stem = (_rectangle(p, padding) for p in parts)
    # Boundary of the union, not two crossing rectangular outlines or a solid T.
    vertices = ((cross[0], cross[3]), (cross[1], cross[3]),
                (cross[1], cross[2]), (stem[1], cross[2]),
                (stem[1], stem[2]), (stem[0], stem[2]),
                (stem[0], cross[2]), (cross[0], cross[2]))
    target = []
    for i, (a, b) in enumerate(zip(vertices, (*vertices[1:], vertices[0]))):
        dx, dy = b[0] - a[0], b[1] - a[1]
        target.append(box(
            f"target_outline_{i}", (math.hypot(dx, dy), 0.002, 0.0004),
            (article.goal[0] + (a[0] + b[0]) / 2,
             article.goal[1] + (a[1] + b[1]) / 2, 0.0002),
            (0.1, 0.65, 0.25), (0, 0, math.atan2(dy, dx)), collision=False,
        ))
    return BenchSpec(
        replace(design("L3-14", "夹爪推 T 形块定姿", (article,), target),
                version="l3-14-gripper-push-v2"),
        facts="Use the LEFT gripper's exterior directly to push the red T block; there is no separate tool. "
              "Keep the fingers together and contact a side of the block; do not pinch, lift or carry it. "
              "The green T is a flat outline with no collision. It is expanded 20 mm on each side, "
              "giving a 140 by 140 mm overall target for the 100 by 100 mm block. "
              "Finish with the entire block footprint inside the outline and its T orientation within "
              "25 degrees of the target. Withdraw both grippers and confirm one second of stability on the table.",
        limitations="Rigid planar gripper pushing with relaxed target containment. Geometric success alone "
                    "does not prove force adaptation or L3-level continuous joint adjustment.",
        parameters={"target_rectangles": (cross, stem), "target_padding_m": padding,
                    "yaw_tolerance_rad": math.radians(25), "push_translation_m": 0.003,
                    "push_rotation_rad": math.radians(3), "max_lift_m": 0.008, "hold_s": 1.0},
    )


def _clip_rectangle(polygon, bounds):
    """Sutherland-Hodgman clipping against four axis-aligned half-planes."""
    for axis, bound, sign in ((0, bounds[0], 1), (0, bounds[1], -1),
                              (1, bounds[2], 1), (1, bounds[3], -1)):
        if not polygon:
            break
        result = []
        previous = polygon[-1]
        previous_distance = sign * (previous[axis] - bound)
        for current in polygon:
            distance = sign * (current[axis] - bound)
            if (distance >= 0) != (previous_distance >= 0):
                fraction = previous_distance / (previous_distance - distance)
                result.append(previous + fraction * (current - previous))
            if distance >= 0:
                result.append(current)
            previous, previous_distance = current, distance
        polygon = result
    return polygon


def _area(polygon):
    if len(polygon) < 3:
        return 0.0
    points = np.asarray(polygon)
    return float(abs(points[:, 0] @ np.roll(points[:, 1], -1)
                     - points[:, 1] @ np.roll(points[:, 0], -1)) / 2)


def _convex_hull(points):
    """Projected box silhouette, including top/bottom corners when tilted."""
    points = sorted(set(map(tuple, points)))

    def half(sequence):
        result = []
        for c in sequence:
            while len(result) >= 2:
                a, b = result[-2:]
                turn = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                if turn > 0:
                    break
                result.pop()
            result.append(c)
        return result[:-1]

    return [np.asarray(p) for p in half(points) + half(reversed(points))]


def footprint_outside_area(article, actual_pose, target_rectangles):
    """Exact projected area outside the T union, including its concave notches.

    Testing only a bounding box or vertices would accept edges crossing a notch.
    Both the rendered outline and this union use the same rectangle bounds.
    """
    transform = np.linalg.inv(pose_transform(article.goal)) @ pose_transform(actual_pose)
    a, b = target_rectangles
    overlap = (max(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), min(a[3], b[3]))
    outside = 0.0
    for part in article.parts:
        xmin, xmax, ymin, ymax = _rectangle(part)
        z = part.position[2]
        corners = np.array([(x, y, h) for x in (xmin, xmax) for y in (ymin, ymax)
                            for h in (z - part.size[2] / 2, z + part.size[2] / 2)])
        polygon = _convex_hull((corners @ transform[:3, :3].T + transform[:3, 3])[:, :2])
        covered = sum(_area(_clip_rectangle(polygon, r)) for r in (a, b))
        if overlap[0] < overlap[1] and overlap[2] < overlap[3]:
            covered -= _area(_clip_rectangle(polygon, overlap))
        outside += max(0.0, _area(polygon) - covered)
    return outside
