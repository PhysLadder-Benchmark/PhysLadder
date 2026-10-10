"""Watertight convex thread pieces. No screw joint or pose/rotation coupling.

Each short trapezoidal helix segment is a separate convex collider, moving on
the male part and static on the female part. The surrounding annular shell is
one static triangle mesh with an open bore. Right-hand pitch: clockwise rotation
viewed from above advances the male screw downwards.
"""

import math

import numpy as np

from .spec import SPEC

# Quad faces for two identical cross-sections; normals are corrected below.
_FACES = (
    (0, 3, 2, 1),
    (4, 5, 6, 7),
    (0, 1, 5, 4),
    (1, 2, 6, 5),
    (2, 3, 7, 6),
    (3, 0, 4, 7),
)


def closed_piece(points):
    points = np.asarray(points, dtype=float)
    faces = []
    center = points.mean(axis=0)
    for quad in _FACES:
        for tri in ((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])):
            a, b, c = points[list(tri)]
            if np.dot(np.cross(b - a, c - a), (a + b + c) / 3 - center) < 0:
                tri = tuple(reversed(tri))
            faces.append(tri)
    return points, np.asarray(faces, dtype=int)


def helix_pieces(*, female=False, spec=SPEC):
    s = spec
    turns = s.socket_thread_turns if female else s.thread_turns
    start = s.socket_thread_start if female else s.thread_start
    n = round(turns * s.segments_per_turn)
    if female:
        # Female inward teeth occupy the spaces BETWEEN the male outward teeth.
        # Crest phase differs by half a pitch; radial and axial clearances are explicit.
        root, crest = s.crest_radius + 0.003, s.core_radius + s.radial_clearance
    else:
        root, crest = s.core_radius - 0.0004, s.crest_radius
    for i in range(n):
        points = []
        for theta in (
            i * 2 * math.pi / s.segments_per_turn,
            (i + 1) * 2 * math.pi / s.segments_per_turn,
        ):
            z = start + s.pitch * theta / (2 * math.pi)
            for radius, dz in (
                (root, -s.thread_root_width / 2),
                (crest, -s.thread_crest_width / 2),
                (crest, s.thread_crest_width / 2),
                (root, s.thread_root_width / 2),
            ):
                points.append(
                    (radius * math.cos(theta), radius * math.sin(theta), z + dz)
                )
        yield closed_piece(points)


def socket_shell_mesh(spec=SPEC):
    s = spec
    pieces = []
    # Open annular shell. Its polygon is circumscribed with a generous bore.
    inner = s.crest_radius + 0.002
    for i in range(64):
        points = []
        for theta in (i * 2 * math.pi / 64, (i + 1) * 2 * math.pi / 64):
            for radius, z in (
                (inner, s.socket_floor),
                (s.socket_radius, s.socket_floor),
                (s.socket_radius, s.socket_rim),
                (inner, s.socket_rim),
            ):
                points.append((radius * math.cos(theta), radius * math.sin(theta), z))
        v, f = closed_piece(points)
        # Weld the wall into one continuous surface without internal end faces.
        pieces.append((v, f[4:]))
    vertices, faces, offset = [], [], 0
    for v, f in pieces:
        vertices.extend(v)
        faces.extend(f + offset)
        offset += len(v)
    vertices, inverse = np.unique(np.round(vertices, 12), axis=0, return_inverse=True)
    return vertices, inverse[np.asarray(faces)]


def screw_surface(spec=SPEC):
    s = spec
    points = [v for v, _ in helix_pieces(spec=spec)]
    angles = np.linspace(0, 2 * math.pi, 32, endpoint=False)
    for radius, levels in (
        (s.core_radius, (0, s.shaft_length)),
        (s.head_radius, (s.shaft_length, s.height)),
    ):
        points.append(
            np.array(
                [
                    (radius * math.cos(a), radius * math.sin(a), z)
                    for a in angles
                    for z in levels
                ]
            )
        )
    return np.concatenate(points)
