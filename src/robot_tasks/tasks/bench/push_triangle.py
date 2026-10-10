"""Native triangular block and an intentionally generous visual target."""

from dataclasses import replace
import math

import numpy as np

from ...robots.transforms import pose_transform
from ..l1.specs import Article, box, pose
from .geometry import design
from .spec import BenchSpec

VERTICES = tuple(
    (x, y, z)
    for z in (0.0, 0.010)
    for x, y in ((0.0, 0.0085), (-0.016, -0.0085), (0.016, -0.0085))
)
TARGET = ((0.0, 0.10), (-0.10, -0.08), (0.10, -0.08))


def build():
    a = Article(
        "triangle",
        (),
        pose(0.30, 1.24, 0.781),
        pose(0.18, 1.10, 0.78),
        0.003,
        asset="tuojing:blue_triangle",
        asset_bounds=((-0.016, -0.0085, 0), (0.016, 0.0085, 0.010)),
    )
    outline = []
    for i, (u, v) in enumerate(zip(TARGET, (*TARGET[1:], TARGET[0]))):
        dx, dy = v[0] - u[0], v[1] - u[1]
        outline.append(
            box(
                f"target_outline_{i}",
                (math.hypot(dx, dy), 0.002, 0.0004),
                (a.goal[0] + (u[0] + v[0]) / 2, a.goal[1] + (u[1] + v[1]) / 2, 0.7802),
                (0.1, 0.65, 0.25),
                (0, 0, math.atan2(dy, dx)),
                collision=False,
            )
        )
    return BenchSpec(
        replace(
            design("L3-14", "夹爪推三角块入宽容目标区", [a], outline),
            version="l3-14-photo-layout-triangle-v4",
            release=False,
            provenance="2026-10-08 blue triangle, original dimensions and collision mesh",
        ),
        arms=("right",),
        mechanism="native_rigid_triangle_pushing",
        facts="Use RIGHT gripper exterior to PUSH the small blue triangular block on the table into the large green triangular outline. LEFT observes only. From the shared overview image the block starts ABOVE-RIGHT of the target, closer to the robots, and both triangles point toward the robots. Push toward the large region below-left; these directions describe the initial image, not fixed wrist axes. The block is 32 x 17 x 10 mm; target is about 200 x 180 mm. The green line is non-colliding and not a second object. Keep fingers together and push a side above the tabletop; do not grasp, lift or carry. The WHOLE block must enter the region on the table. Any in-plane orientation is accepted; no exact centre, release or timed wait is required.",
        limitations="Rigid planar pushing with a generous nonphysical target. This simplified geometric task alone does not establish L3 force adaptation; native friction is estimated.",
        parameters={
            "table_z": 0.78,
            "hold_s": 0.0,
            "target_vertices": TARGET,
            "block_vertices": VERTICES,
            "push_translation_m": 0.003,
            "push_rotation_rad": math.radians(3),
            "max_lift_m": 0.008,
            "native_instances": [
                {
                    "name": "triangle",
                    "asset": "blue_triangle",
                    "pose": a.initial,
                    "tracked": {"triangle": ""},
                }
            ],
        },
    )


def containment_margin(actual_pose, goal, vertices=VERTICES, target=TARGET):
    """Signed metres to convex target sides; actual prism vertices, not an AABB.

    The very same CCW polygon is used for the rendered outline. Negative means
    some part protrudes; centre-only or enclosing-rectangle tests are insufficient.
    """
    t = np.linalg.inv(pose_transform(goal)) @ pose_transform(actual_pose)
    points = (np.asarray(vertices) @ t[:3, :3].T + t[:3, 3])[:, :2]
    margins = []
    for a, b in zip(target, (*target[1:], target[0])):
        edge = np.subtract(b, a)
        d = points - a
        margins.extend((edge[0] * d[:, 1] - edge[1] * d[:, 0]) / np.linalg.norm(edge))
    return float(min(margins))
