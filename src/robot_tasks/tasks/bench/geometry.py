"""Open hollow solids made of convex pieces; no hull seals a visible aperture."""

import math
from dataclasses import replace

import numpy as np

from ..l1.specs import BLUE, RED, Part, box
from ..screw.geometry import closed_piece


def cylinder(name, radius, height, xyz=(0, 0, 0), color=RED, rpy=(0, 0, 0)):
    return Part(
        name, (radius, radius, height), tuple(xyz), tuple(rpy), "cylinder", color
    )


def cup_parts(
    name="cup",
    *,
    bottom_radius=0.022,
    top_radius=0.030,
    height=0.065,
    wall=0.003,
    floor=0.004,
    color=BLUE,
    segments=24,
):
    """Origin is the bottom centre; radii describe the inner cavity."""
    parts = [
        cylinder(name + "_floor", bottom_radius + wall, floor, (0, 0, floor / 2), color)
    ]
    for i in range(segments):
        points = []
        for angle in (i * 2 * math.pi / segments, (i + 1) * 2 * math.pi / segments):
            for radius, z in (
                (bottom_radius, floor),
                (bottom_radius + wall, floor),
                (top_radius + wall, height),
                (top_radius, height),
            ):
                points.append((radius * math.cos(angle), radius * math.sin(angle), z))
        vertices, faces = closed_piece(points)
        parts.append(
            Part(
                f"{name}_wall_{i}",
                (0, 0, 0),
                shape="mesh",
                color=color,
                vertices=tuple(tuple(float(v) for v in row) for row in vertices),
                faces=tuple(tuple(int(v) for v in row) for row in faces),
            )
        )
    return tuple(parts)


def translated(parts, xyz, prefix=""):
    return tuple(
        replace(p, name=prefix + p.name, position=tuple(np.asarray(p.position) + xyz))
        for p in parts
    )


def placed_parts(parts, xyz, yaw):
    """Place a local fixture assembly with the same yaw as its moving props."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple(
        replace(p, position=(xyz[0] + c * p.position[0] - s * p.position[1],
                             xyz[1] + s * p.position[0] + c * p.position[1],
                             xyz[2] + p.position[2]),
                rotation=(p.rotation[0], p.rotation[1], p.rotation[2] + yaw))
        for p in parts
    )


def tray_parts(
    name, width=0.16, depth=0.12, wall_height=0.035, floor=0.004, color=BLUE
):
    return (
        box(name + "_floor", (width, depth, floor), (0, 0, floor / 2), color),
        box(
            name + "_back",
            (width, 0.004, wall_height),
            (0, -depth / 2, wall_height / 2),
            color,
        ),
        box(
            name + "_left",
            (0.004, depth, wall_height),
            (-width / 2, 0, wall_height / 2),
            color,
        ),
        box(
            name + "_right",
            (0.004, depth, wall_height),
            (width / 2, 0, wall_height / 2),
            color,
        ),
    )


def floor_target(xy, size=(0.12, 0.12), name="target"):
    return box(name, (*size, 0.0004), (*xy, 0.0002), (0.1, 0.65, 0.2), collision=False)


def design(task_id, name, articles, fixtures=(), *, friction=0.35):
    from ..l1.specs import Design

    return Design(
        task_id,
        name,
        "household",
        tuple(articles),
        tuple(fixtures),
        "",
        version=task_id.lower() + "-household-v1",
        fixture_friction=friction,
    )
