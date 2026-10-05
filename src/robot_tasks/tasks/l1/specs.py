"""Pinned and parameterized rigid test articles; explicit l1-rigid-v1 variants.

All dimensions are metres. Holes are unions of wall colliders, never a convex
hull enclosing a cavity. This module imports neither Isaac nor model clients.
"""

import math
from dataclasses import dataclass, replace

import numpy as np

from ...robots.transforms import origin_transform, transform_pose
from .cylinder_insertion import CylinderInsertion
from .shaped_recess import ShapedRecess

RED = (0.85, 0.16, 0.05)
BLUE = (0.08, 0.3, 0.8)
GREY = (0.38, 0.43, 0.48)
GREEN = (0.08, 0.65, 0.25)
YELLOW = (0.95, 0.7, 0.08)


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return transform_pose(origin_transform((x, y, z), (roll, pitch, yaw)))


@dataclass(frozen=True)
class Part:
    name: str
    size: tuple[float, float, float]
    position: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = (0.0, 0.0, 0.0)
    shape: str = "box"
    color: tuple = RED
    collision: bool = True
    vertices: tuple = ()
    faces: tuple = ()


@dataclass(frozen=True)
class Article:
    name: str
    parts: tuple[Part, ...]
    initial: tuple
    goal: tuple
    mass: float = 0.06
    asset: str = ""
    asset_size: tuple = ()
    asset_rotation: tuple = (0.0, 0.0, 0.0)
    asset_bounds: tuple = ()  # Optional (min_xyz, max_xyz), for native uncentred metre assets.


@dataclass(frozen=True)
class Design:
    task_id: str
    name: str
    kind: str
    articles: tuple[Article, ...]
    fixtures: tuple[Part, ...]
    instruction: str
    fixture_top: float = 0.0
    clearance: float = 0.005
    release: bool = True
    fixture_asset: str = ""
    fixture_size: tuple = ()
    fixture_pose: tuple = ()
    provenance: str = "Project-authored parametric rigid test article v1"
    version: str = "l1-rigid-v1"
    insertion: CylinderInsertion | ShapedRecess | None = None
    fixture_friction: float = 0.8


def box(name, size, xyz=(0.0, 0.0, 0.0), color=GREY, rpy=(0.0, 0.0, 0.0), collision=True):
    return Part(name, tuple(size), tuple(xyz), tuple(rpy), color=color, collision=collision)


def rectangular_well(center, opening, height, bottom=0.006):
    x, y = center
    a, b = opening
    wall = 0.016
    return (
        box("floor", (a + 2 * wall, b + 2 * wall, bottom), (x, y, bottom / 2)),
        box("west", (wall, b + 2 * wall, height), (x - a / 2 - wall / 2, y, bottom + height / 2)),
        box("east", (wall, b + 2 * wall, height), (x + a / 2 + wall / 2, y, bottom + height / 2)),
        box("south", (a, wall, height), (x, y - b / 2 - wall / 2, bottom + height / 2)),
        box("north", (a, wall, height), (x, y + b / 2 + wall / 2, bottom + height / 2)),
    )


def ring_parts(radius=0.025, tube=0.008, count=32, color=RED):
    # Tangential boxes overlap slightly, retaining a real open centre.
    return tuple(
        box(
            f"segment_{i}",
            (2 * radius * math.tan(math.pi / count) * 1.04, 2 * tube, 2 * tube),
            (radius * math.cos(a), radius * math.sin(a), 0.0),
            color,
            (0.0, 0.0, a + math.pi / 2),
        )
        for i, a in enumerate(np.arange(count) * 2 * math.pi / count)
    )


def _procedural_design(task_id):
    source = (0.36, 0.44)
    target = (0.36, 0.25)
    x, y = target
    if task_id == "L1-01":
        g = CylinderInsertion()
        parts = (
            Part("peg", (g.radius_m, g.radius_m, g.height_m), (0, 0, 0), shape="cylinder"),
        )
        # Tangent walls form an open circumscribed polygon. Never use one convex
        # collider around the socket: that would seal its visible opening.
        radius = g.hole_radius_m + g.wall_thickness_m / 2
        tangential = 2 * (g.hole_radius_m + g.wall_thickness_m) * math.tan(math.pi / g.wall_segments)
        fixtures = [box("floor", (0.085, 0.085, g.floor_thickness_m),
                        (x, y, g.floor_thickness_m / 2), BLUE)]
        for i in range(g.wall_segments):
            angle = i * 2 * math.pi / g.wall_segments
            fixtures.append(box(f"wall_{i}", (g.wall_thickness_m, tangential, g.hole_depth_m),
                                (x + radius * math.cos(angle), y + radius * math.sin(angle),
                                 g.floor_thickness_m + g.hole_depth_m / 2),
                                BLUE, (0, 0, angle)))
        article = Article("cylinder", parts, pose(0.36, 0.40, g.height_m / 2 + 0.001),
                          pose(x, y, g.floor_thickness_m + g.height_m / 2))
        return Design(
            task_id, "圆柱插孔（等径宽容间隙）", "insert", (article,), tuple(fixtures),
            f"Insert the orange uniform cylindrical peg into the single opening of the fixed blue socket. "
            f"The peg is {2*g.radius_m:.3f} m in diameter throughout and {g.height_m:.3f} m tall. "
            "Both flat ends are equivalent; either may enter the socket. Choose a grasp that leaves "
            "the entering end and the fingers' approach clear of the socket rim. "
            f"The socket opening is {2*g.hole_radius_m:.3f} m across and {g.hole_depth_m:.3f} m deep. "
            f"After lifting, reach at least {g.required_depth_m:.3f} m insertion depth while grasping the peg; "
            "then leave it seated inside the socket without robot support, with its upper end above the rim. "
            "Peg/socket friction is zero; gravity, normal collisions and gripper friction remain enabled. "
            "No force adjustment, press fit or deformation is required.",
            fixture_top=g.rim_z_m, clearance=g.hole_radius_m-g.radius_m,
            provenance="Project-authored single-socket rigid peg; not the original RoboDojo insert_tubes task",
            version="l1-01-wide-clearance-v3", insertion=g, fixture_friction=0.0,
        )
    if task_id == "L1-02":
        g = ShapedRecess()
        vertices, faces = g.block_mesh()
        block = Part("body", (0.033, 0.050, g.height_m), shape="mesh",
                     color=RED, vertices=vertices, faces=faces)
        fixtures = [box("floor", (0.09, 0.09, g.floor_thickness_m),
                        (x, y, g.floor_thickness_m / 2), BLUE)]
        for i, (vertices, faces) in enumerate(g.wall_meshes()):
            fixtures.append(Part(f"wall_{i}", (0, 0, 0), (x, y, 0), shape="mesh",
                                 color=BLUE, vertices=vertices, faces=faces))
        article = Article("d_block", (block,), pose(0.36, 0.40, g.height_m / 2 + 0.001, yaw=math.pi / 4),
                          pose(x, y, g.floor_thickness_m + g.height_m / 2))
        return Design(
            task_id, "D 形块嵌浅槽（宽容间隙）", "insert", (article,), tuple(fixtures),
            "Fit the orange D-shaped rigid block into the matching single blue shallow recess. "
            "The profile is a circle with one side cut flat: match the flat side as well as the curved side. "
            "The block is about 0.050 m across its widest direction, 0.033 m from flat side to round tip, "
            "and 0.040 m tall. The recess is 0.012 m deep above a 0.006 m floor, with 0.004 m "
            "side clearance and a chamfered mouth. About 0.028 m of block remains above the rim when seated. "
            "The fixture contact friction is zero; gripper friction, gravity, collisions and support remain. "
            "No press fit, force adjustment, snap fit or deformation is required. There is only one block "
            "and one recess; identify their matching orientation from observations.",
            fixture_top=g.rim_z_m, clearance=g.clearance_m,
            provenance="Project-authored D profile and shallow recess; not a ManiSkill asset or score",
            version="l1-02-d-recess-v2", insertion=g, fixture_friction=0.0,
        )
    if task_id == "L1-04":
        # Closed receiving box; only the slit in the lid admits the plate.
        fixtures = list(rectangular_well(target, (0.12, 0.12), 0.09))
        z = 0.100
        opening = (0.016, 0.078)
        extent = 0.152
        thick = 0.008
        fixtures += [
            box(
                "lid_left", ((extent - opening[0]) / 2, extent, thick), (x - (extent + opening[0]) / 4, y, z)
            ),
            box(
                "lid_right", ((extent - opening[0]) / 2, extent, thick), (x + (extent + opening[0]) / 4, y, z)
            ),
            box(
                "lid_front",
                (opening[0], (extent - opening[1]) / 2, thick),
                (x, y - (extent + opening[1]) / 4, z),
            ),
            box(
                "lid_back",
                (opening[0], (extent - opening[1]) / 2, thick),
                (x, y + (extent + opening[1]) / 4, z),
            ),
        ]
        part = box("plate", (0.006, 0.066, 0.060), color=RED)
        article = Article(
            "plate",
            (part,),
            pose(*source, 0.003, pitch=math.pi / 2),
            pose(x, y, 0.009, pitch=math.pi / 2),
            0.03,
        )
        return Design(
            task_id,
            "薄片入槽",
            "slot",
            (article,),
            tuple(fixtures),
            "Pick up the rigid red plate and pass it completely through the narrow opening in the grey box lid. It must end entirely inside the receiving box after release. Do not bypass the slit.",
            z,
            0.005,
        )
    if task_id == "L1-06":
        parts = (
            box("handle", (0.09, 0.020, 0.024), (-0.025, 0, 0), BLUE),
            box("working_head", (0.046, 0.064, 0.024), (0.043, 0, 0), RED),
        )
        fixtures = (
            box("target", (0.16, 0.12, 0.001), (x, y, 0.0005), GREEN, collision=False),
            box("direction", (0.065, 0.006, 0.002), (x + 0.04, y, 0.002), YELLOW, collision=False),
        )
        article = Article("tool", parts, pose(*source, 0.012, yaw=0.55), pose(x, y, 0.012))
        return Design(
            task_id,
            "工具柄定点抓取",
            "tool",
            (article,),
            fixtures,
            "Grasp the blue handle of the scraper, never the red working head. Lift it stably with the working head pointing along world +X (the yellow line), then place it flat on the green patch with that same direction. Release and withdraw. Both finger contacts must be on the blue handle.",
            clearance=0.010,
        )
    if task_id == "L1-07":
        fixtures = (
            box("foot", (0.075, 0.06, 0.012), (x, y - 0.05, 0.006)),
            box("post", (0.012, 0.012, 0.14), (x, y - 0.05, 0.082)),
            box("prong", (0.008, 0.074, 0.008), (x, y - 0.013, 0.142)),
            box("lip", (0.008, 0.008, 0.020), (x, y + 0.020, 0.156)),
        )
        # Ring lies flat initially; normal becomes +Y when hanging on the prong.
        article = Article(
            "ring",
            ring_parts(),
            pose(*source, 0.008),
            pose(x, y, 0.142 + 0.004 - 0.017, roll=math.pi / 2),
            0.04,
        )
        return Design(
            task_id,
            "挂环上钩",
            "ring",
            (article,),
            fixtures,
            "Pick up the rigid red ring without obstructing its hole. Thread it over the free upturned tip of the grey hook and hang it on the horizontal prong. Release and withdraw so the hook alone supports the ring.",
            0.142,
            0.010,
        )
    if task_id == "L1-08":
        base = (
            box("body", (0.060, 0.060, 0.028), color=BLUE),
            box("stripe", (0.050, 0.004, 0.001), (0, 0, 0.0146), YELLOW, collision=False),
        )
        top = (
            box("body", (0.050, 0.032, 0.028), color=RED),
            box("stripe", (0.044, 0.004, 0.001), (0, 0, 0.0146), YELLOW, collision=False),
        )
        articles = (
            Article("base_block", base, pose(0.32, 0.46, 0.014, yaw=0.3), pose(x, y, 0.014)),
            Article("top_block", top, pose(0.44, 0.43, 0.014, yaw=-0.3), pose(x, y, 0.042, yaw=math.pi / 2)),
        )
        fixtures = (
            box("target", (0.085, 0.085, 0.001), (x, y, 0.0005), GREEN, collision=False),
            box("direction", (0.05, 0.004, 0.001), (x + 0.065, y, 0.001), YELLOW, collision=False),
        )
        return Design(
            task_id,
            "刚性积木定姿堆叠",
            "stack",
            articles,
            fixtures,
            "Build a two-block tower centred on the green patch: blue block on the table, red block centred on top. The blue yellow stripe must point along world X, parallel to the yellow table line; the red stripe must point along world Y. Release both blocks and withdraw; the tower must stand independently.",
            clearance=0.010,
        )
    raise ValueError(f"Unsupported rigid L1 task: {task_id}")


RIGID_IDS = tuple(f"L1-{i:02}" for i in range(1, 9))
FILES = (
    "cylinder_insertion",
    "shaped_recess",
    "plug_insertion",
    "plate_slot",
    "key_insertion",
    "tool_handle",
    "ring_hook",
    "block_stack",
)


def build_design(task_id):
    """Explicit asset choices, identical on every OS; no runtime fallbacks."""
    if task_id not in ("L1-03", "L1-05"):
        return _procedural_design(task_id)
    names = {"L1-03": "插头插接", "L1-05": "钥匙插锁"}
    d = Design(task_id, names[task_id], "insert", (), (), "", release=task_id != "L1-05")
    if task_id == "L1-03":
        a = Article(
            "plug",
            (),
            pose(0.36, 0.44, 0.0136, roll=-math.pi / 2),
            pose(0.36, 0.25, 0.0373),
            0.03,
            "Rigid/charger/00000",
            (0.042, 0.027004, 0.063075),
            (math.pi / 2, 0, 0),
        )
        return replace(
            d,
            articles=(a,),
            fixtures=(),
            fixture_asset="Rigid/socket/00000",
            fixture_size=(0.15719, 0.036329, 0.020804),
            fixture_pose=pose(0.31645, 0.2437, 0.010402),
            fixture_top=0.020804,
            clearance=0.003,
            instruction="Pick up the charger and insert both metal pins into the socket pair at the world +X end of the fixed power strip. This is unpowered, with no latch. Seat the plug fully with its body above the strip, release and withdraw. Do not merely rest it on top of the strip.",
            provenance="RoboDojo charger/00000 + socket/00000; fixed socket; CPU collision adaptation v1",
        )
    if task_id == "L1-05":
        a = Article(
            "key",
            (),
            pose(0.36, 0.44, 0.0051, roll=math.pi / 2),
            pose(0.36, 0.25, 0.0566),
            0.04,
            "Rigid/key/00000",
            (0.0227104, 0.010000, 0.0628467),
        )
        return replace(
            d,
            articles=(a,),
            fixtures=(),
            fixture_asset="Geometry/key_slot/00000",
            fixture_size=(0.100, 0.100, 0.055),
            fixture_pose=pose(0.36, 0.25, 0.0275),
            fixture_top=0.055,
            clearance=0.003,
            instruction="Pick up the key by its round head and insert the toothed shaft into the fixed lock opening, matching the keyway orientation. Insert the shaft fully with its tip near the bottom. Do not turn the key. You may keep holding its head after insertion.",
            provenance="RoboDojo key/00000 + key_slot/00000; insert-only variant v1",
        )
    return d


def local_corners(article):
    """Conservative visual/collision envelope for process and containment checks."""
    from itertools import product

    if article.asset:
        if article.asset_bounds:
            return np.array(list(product(*zip(*article.asset_bounds))))
        return np.array(list(product(*[(-s / 2, s / 2) for s in article.asset_size])))
    points = []
    for part in article.parts:
        if not part.collision:
            continue
        if part.shape == "mesh":
            t = origin_transform(part.position, part.rotation)
            points.extend(np.asarray(part.vertices) @ t[:3, :3].T + t[:3, 3])
            continue
        size = part.size if part.shape == "box" else (2 * part.size[0], 2 * part.size[1], part.size[2])
        corners = np.array(list(product(*[(-s / 2, s / 2) for s in size])))
        t = origin_transform(part.position, part.rotation)
        points.extend(corners @ t[:3, :3].T + t[:3, 3])
    return np.asarray(points)
