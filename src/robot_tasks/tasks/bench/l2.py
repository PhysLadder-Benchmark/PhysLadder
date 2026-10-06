"""Passive support, natural sliding resistance and a declared elastic proxy."""

import itertools
import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, Part, box, pose
from .geometry import cup_parts, design, floor_target, placed_parts
from .spec import BenchSpec, Joint


def books():
    # Adjacent books are fixed fixtures; only the protruding middle book moves.
    # Longitudinal stack, front toward the left arm. Keep room for the full pull.
    x, y, yaw = 0.42, 0.34, 0.0
    middle = Article("middle_book", (box("cover", (0.12, 0.045, 0.024), color=RED),),
                     pose(x - 0.085, y, 0.066, yaw=yaw),
                     pose(x - 0.165, y, 0.066, yaw=yaw), 0.09)
    fixtures = placed_parts((
        box("lower_book", (0.16, 0.056, 0.026), (0, 0, 0.040), BLUE),
        box("upper_book", (0.16, 0.056, 0.026), (0, 0, 0.093), GREEN),
        box("support_front", (0.022, 0.070, 0.027), (-0.060, 0, 0.0135), GREY),
        box("support_back", (0.022, 0.070, 0.027), (0.060, 0, 0.0135), GREY),
    ), (x, y, 0), yaw)
    return BenchSpec(
        replace(design("L2-03", "固定上下层的单臂抽书", [middle], fixtures),
                version="l2-03-fixed-stack-v4-front", release=False),
        mechanism="fixed_adjacent_books_and_free_middle_book",
        limitations="Upper/lower rigid books and supports are fixed. Only middle-book contact friction and extraction are tested; upper-book support/stability is excluded.",
        facts="The three books remain stacked horizontally on top of each other, with their long axes along world X. The red middle book is 120 x 45 x 24 mm and protrudes toward the LEFT arm (world -X). Upper green and lower blue books are fixed fixtures. Grasp the exposed front with the left fingers closing across its 45 mm width (world Y), then withdraw horizontally along -X until its entire rear edge clears the stack. Do not stand the books upright or pull along Y. Keep holding it still for one simulation second; placement and release are not required. The right arm may observe only.",
        parameters={"stack_front_world": (x - 0.080, y, 0), "clearance_m": 0.004,
                    "minimum_travel_m": 0.065, "outward_axis_world": (-1.0, 0.0, 0.0)},
    )


def bowl():
    # Dense, initially contained spheres. Gravity/contact settle the packing;
    # there is no feeder, attachment, or prescribed ball animation.
    source, target = (0.38, 0.12, 0.001), (0.38, -0.12, 0.001)
    bottom, top, height, radius, spacing = 0.050, 0.070, 0.064, 0.010, 0.0205
    parts = cup_parts(bottom_radius=bottom, top_radius=top, height=height, color=BLUE) + (
        box("handle_left", (0.040, 0.050, 0.020), (0, 0.090, 0.040), BLUE),
        box("handle_right", (0.040, 0.050, 0.020), (0, -0.090, 0.040), BLUE),
    )
    articles = [Article("bowl", parts, pose(*source), pose(*target), 0.12)]
    for layer in range(3):
        z = 0.004 + radius + layer * spacing * math.sqrt(2 / 3)
        offset = (spacing / 2, spacing * math.sqrt(3) / 6) if layer % 2 else (0, 0)
        safe_radius = (bottom + (top - bottom) * (z - radius) / height) * math.cos(math.pi / 24) - radius - 0.001
        for i, j in itertools.product(range(-3, 4), repeat=2):
            x = spacing * (i + j / 2) + offset[0]
            y = spacing * math.sqrt(3) / 2 * j + offset[1]
            if math.hypot(x, y) > safe_radius:
                continue
            index = len(articles) - 1
            ball = Part("sphere", (radius, radius, 2 * radius), shape="sphere", color=RED)
            initial = pose(source[0] + x, source[1] + y, source[2] + z)
            goal = pose(target[0] + x, target[1] + y, target[2] + z)
            articles.append(Article(f"load_{index}", (ball,), initial, goal, 0.004))
    count = len(articles) - 1
    return BenchSpec(
        replace(design("L2-04", "满载低摩擦小球的碗搬运", articles,
                       [floor_target(target[:2], (0.17, 0.17))]), version="l2-04-preloaded-balls-v2"),
        arms=("left", "right"),
        mechanism="preloaded_low_friction_spheres_transport",
        limitations="Rigid low-friction spheres in a rigid bowl; no liquid or material damage. Ball contact friction is 0.03; bowl/handle grip friction remains unchanged.",
        facts=f"The blue bowl already contains {count} separate red spheres, each 20 mm across. They roll and slide freely with low contact friction. Use both R5 hands on the two external handles. Lift the loaded bowl from its initial table location, carry it to the green patch without spilling, set it upright on the patch and release both hands. The bowl centre must be within 35 mm of the patch centre; all balls must remain inside, and the released bowl must be still for one simulation second. There is no feeder or extra loading phase.",
        parameters={"target_xy": target[:2], "target_tolerance_m": 0.035,
                    "count": count, "ball_radius_m": radius, "ball_friction": 0.03,
                    "bottom_radius_m": bottom, "top_radius_m": top, "height_m": height,
                    "floor_m": 0.004, "wall_segments": 24, "lift_m": 0.025},
    )


def jelly():
    # Eight compliant cells joined by passive 6-DOF springs; no rigid grab attachment.
    vertices = list(itertools.product((-0.010, 0.010), repeat=3))
    articles = [
        Article(
            f"jelly_{i}",
            (box("cell", (0.022, 0.022, 0.022), color=(0.7, 0.16, 0.5)),),
            pose(0.36 + x, 0.43 + y, 0.022 + z),
            pose(0.36 + x, 0.22 + y, 0.022 + z),
            0.010,
        )
        for i, (x, y, z) in enumerate(vertices)
    ]
    joints = []
    for i, j in itertools.combinations(range(8), 2):
        a, b = articles[i].initial[:3], articles[j].initial[:3]
        anchor = tuple((x + y) / 2 for x, y in zip(a, b))
        axes = {
            axis: (-0.012, 0.012, 35.0, 0.15) for axis in ("transX", "transY", "transZ")
        }
        axes.update(
            {axis: (-45, 45, 0.0005, 0.0005) for axis in ("rotX", "rotY", "rotZ")}
        )
        joints.append(
            Joint(f"elastic_{i}_{j}", f"jelly_{i}", f"jelly_{j}", anchor, axes)
        )
    return BenchSpec(
        design(
            "L2-06", "夹果冻（弹性格点代理）", articles, [floor_target((0.36, 0.22))]
        ),
        joints=tuple(joints),
        mechanism="spring_lattice_deformation_proxy",
        limitations="Eight-cell elastic proxy with strain threshold; NOT gelatin material calibration, irreversible crushing, fracture, tearing or food damage simulation.",
        facts="Move the exposed purple elastic object onto the green patch and release it. The object is about 0.042 m across. Close gently and inspect deformation; it is not in a rigid container. This simulation uses an elastic lattice with a 35% strain limit, not a model of real jelly cracking.",
        parameters={
            "target_xy": (0.36, 0.22),
            "target_half": 0.06,
            "strain_limit": 0.35,
            "rest_points": [a.initial[:3] for a in articles],
        },
    )


def drawer():
    # Fixed cabinet opens toward the left arm, with an exposed handle and pull room.
    x, y, z, yaw = 0.44, 0.34, 0.014, 0.0
    tray = (
        box("floor", (0.14, 0.12, 0.004), (0, 0, 0.002), BLUE),
        box("side_left", (0.14, 0.004, 0.040), (0, -0.058, 0.022), BLUE),
        box("side_right", (0.14, 0.004, 0.040), (0, 0.058, 0.022), BLUE),
        box("back", (0.004, 0.12, 0.040), (0.068, 0, 0.022), BLUE),
        box("front", (0.008, 0.13, 0.052), (-0.074, 0, 0.026), RED),
        box("handle_stem", (0.032, 0.014, 0.014), (-0.094, 0, 0.055), GREEN),
        box("handle", (0.040, 0.040, 0.026), (-0.125, 0, 0.055), GREEN),
    )
    article = Article("drawer", tray, pose(x, y, z, yaw=yaw), pose(x - 0.11, y, z, yaw=yaw), 0.13)
    fixtures = placed_parts((
        box("rail_bottom", (0.32, 0.15, 0.012), (-0.07, 0, 0.006), GREY),
        box("cabinet_left", (0.18, 0.008, 0.084), (0, -0.075, 0.042), BLUE),
        box("cabinet_right", (0.18, 0.008, 0.084), (0, 0.075, 0.042), BLUE),
        box("cabinet_back", (0.008, 0.15, 0.084), (0.09, 0, 0.042), BLUE),
        box("cabinet_roof", (0.19, 0.158, 0.008), (0, 0, 0.084), BLUE),
    ), (x, y, 0), yaw)
    joint = Joint("drawer_slide", "", "drawer", (x, y, z),
                  {"transX": (-0.13, 0, 0, 0.15), "transZ": (-0.004, 0.002, 0, 0)})
    return BenchSpec(
        replace(design("L2-07", "单臂拉开固定化妆抽屉", [article], fixtures, friction=0.25),
                version="l2-07-pull-only-v4-front"),
        joints=(joint,),
        mechanism="prismatic_rail_and_contact_friction",
        facts="The cosmetics cabinet has a fixed bottom, roof, sides and rear. Its green handle faces the LEFT arm (world -X). Grasp across the handle's 40 mm width (world Y) and pull along world -X by 100-120 mm; maximum travel is 130 mm. Do not pull along Y. Release and leave it open and still for one simulation second. Do not close it again or remove it from its rails. The right arm observes only; the cabinet needs no hand support.",
        parameters={"outward_axis_world": (-1.0, 0.0, 0.0), "required_open_m": 0.10,
                    "max_travel_m": 0.13, "max_speed_m_s": 0.25},
    )


BUILDERS = {
    "L2-03": books,
    "L2-04": bowl,
    "L2-06": jelly,
    "L2-07": drawer,
}
