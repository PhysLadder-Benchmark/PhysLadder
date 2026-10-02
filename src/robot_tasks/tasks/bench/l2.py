"""Passive support, natural sliding/latch resistance and a declared elastic proxy."""

import itertools
import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, Part, box, pose
from .geometry import cup_parts, design, floor_target, translated
from .spec import BenchSpec, Joint


def books():
    # Adjacent books are fixed fixtures; only the protruding middle book moves.
    x, y = 0.42, 0.34
    middle = Article("middle_book", (box("cover", (0.12, 0.045, 0.024), color=RED),),
                     pose(0.335, y, 0.066), pose(0.255, y, 0.066), 0.09)
    fixtures = (
        box("lower_book", (0.16, 0.056, 0.026), (x, y, 0.040), BLUE),
        box("upper_book", (0.16, 0.056, 0.026), (x, y, 0.093), GREEN),
        box("support_front", (0.022, 0.070, 0.027), (0.36, y, 0.0135), GREY),
        box("support_back", (0.022, 0.070, 0.027), (0.48, y, 0.0135), GREY),
    )
    return BenchSpec(
        replace(design("L2-03", "固定上下层的单臂抽书", [middle], fixtures),
                version="l2-03-fixed-stack-v2", release=False),
        mechanism="fixed_adjacent_books_and_free_middle_book",
        limitations="Upper/lower rigid books and supports are fixed. Only middle-book contact friction and extraction are tested; upper-book support/stability is excluded.",
        facts="The red middle book is 120 x 45 x 24 mm and protrudes toward the LEFT robot (world -X). Upper green and lower blue books are fixed fixtures. Grasp the exposed front with the left fingers closing across its 45 mm width, then withdraw horizontally along -X until its entire rear edge clears the stack. Keep holding it still for one simulation second; placement and release are not required. The right arm may observe only.",
        parameters={"stack_front_x": x - 0.080, "clearance_m": 0.004,
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


def latch():
    # Front faces -X, hinge is at the rear (+X). The latch itself is a handle:
    # the same hand retracts it and lifts, then opens beyond vertical to rest.
    x, y = 0.43, 0.34
    walls = (
        box("floor", (0.17, 0.13, 0.006), (0, 0, 0.003), BLUE),
        box("left_wall", (0.17, 0.006, 0.05), (0, -0.063, 0.025), BLUE),
        box("right_wall", (0.17, 0.006, 0.05), (0, 0.063, 0.025), BLUE),
        box("rear_wall", (0.006, 0.126, 0.05), (0.082, 0, 0.025), BLUE),
        box("front_wall", (0.006, 0.126, 0.05), (-0.082, 0, 0.025), BLUE),
        box("catch", (0.012, 0.050, 0.006), (-0.088, 0, 0.047), GREY),
    )
    lid_parts = (
        box("panel", (0.17, 0.13, 0.006), color=BLUE),
        box("grip", (0.028, 0.030, 0.022), (0.048, 0, 0.014), GREEN),
    )
    latch_parts = (
        box("button", (0.028, 0.040, 0.032), color=RED),
        box("latch_stem", (0.024, 0.014, 0.008), (-0.020, 0, -0.004), RED),
        box("tooth", (0.012, 0.032, 0.006), (-0.033, 0, -0.012), RED),
    )
    articles = [
        Article("lid", lid_parts, pose(x, y, 0.058, yaw=math.pi), pose(x, y, 0.058, yaw=math.pi), 0.06),
        Article("latch", latch_parts, pose(x - 0.120, y, 0.051, yaw=math.pi),
                pose(x - 0.120, y, 0.051, yaw=math.pi), 0.008),
        Article("eraser", (box("body", (0.028, 0.040, 0.018), color=(0.95, 0.75, 0.18)),),
                pose(x, y, 0.015), pose(0.28, 0.48, 0.009), 0.025),
    ]
    joints = (
        Joint("lid_hinge", "", "lid", (x + 0.085, y, 0.055), {"rotY": (0, 115, 0, 0.005)}),
        Joint("latch_spring", "lid", "latch", (x - 0.120, y, 0.051),
              {"transX": (-0.020, 0, 60, 0.1)}),
    )
    return BenchSpec(
        replace(design("L2-05", "单臂弹性卡扣翻盖盒开合", articles, translated(walls, (x, y, 0))),
                version="l2-05-single-arm-front-v2"),
        joints=joints,
        mechanism="hinge_and_passive_spring_latch",
        limitations="Fixed box; enlarged spring-slider latch used as a lifting handle. Lid opens past vertical to its gravity-supported 115 degree stop. Not a calibrated moulded flexure; no fracture model.",
        facts="The fixed box already faces the LEFT arm; do not rotate or hold its body. Use the left hand to grasp the red latch handle, retract it toward world -X about 15 mm, then lift the lid through the SAME grasp while keeping the tooth clear. Open past vertical to the 115 degree rear stop; gravity can support the lid there so the hand can release and remove the yellow eraser. Put the eraser outside the box. Regrasp the red latch, keep it retracted as you lower the lid, then release to let the spring re-engage. The right arm may inspect but must not support the box or lid.",
        parameters={"box_xy": (x, y), "latch_closed_x_local": 0.120,
                    "hinge_anchor": (x + 0.085, y, 0.055), "open_stop_deg": 115.0},
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
    # A roofed cosmetics cabinet, fixed to the table, opens toward the robot.
    x, y, z = 0.44, 0.34, 0.014
    tray = (
        box("floor", (0.14, 0.12, 0.004), (0, 0, 0.002), BLUE),
        box("side_left", (0.14, 0.004, 0.040), (0, -0.058, 0.022), BLUE),
        box("side_right", (0.14, 0.004, 0.040), (0, 0.058, 0.022), BLUE),
        box("back", (0.004, 0.12, 0.040), (0.068, 0, 0.022), BLUE),
        box("front", (0.008, 0.13, 0.052), (-0.074, 0, 0.026), RED),
        box("handle_stem", (0.032, 0.014, 0.014), (-0.094, 0, 0.055), GREEN),
        box("handle", (0.040, 0.040, 0.026), (-0.125, 0, 0.055), GREEN),
    )
    article = Article("drawer", tray, pose(x, y, z), pose(x - 0.11, y, z), 0.13)
    fixtures = (
        box("rail_bottom", (0.32, 0.15, 0.012), (x - 0.07, y, 0.006), GREY),
        box("cabinet_left", (0.18, 0.008, 0.084), (x, y - 0.075, 0.042), BLUE),
        box("cabinet_right", (0.18, 0.008, 0.084), (x, y + 0.075, 0.042), BLUE),
        box("cabinet_back", (0.008, 0.15, 0.084), (x + 0.09, y, 0.042), BLUE),
        box("cabinet_roof", (0.19, 0.158, 0.008), (x, y, 0.084), BLUE),
    )
    joint = Joint("drawer_slide", "", "drawer", (x, y, z),
                  {"transX": (-0.13, 0, 0, 0.15), "transZ": (-0.004, 0.002, 0, 0)})
    return BenchSpec(
        replace(design("L2-07", "单臂拉开固定化妆抽屉", [article], fixtures, friction=0.25),
                version="l2-07-front-pull-only-v2"),
        joints=(joint,),
        mechanism="prismatic_rail_and_contact_friction",
        facts="The cosmetics cabinet has a fixed bottom, roof, sides and rear. Its exposed green handle already faces the LEFT arm. Grasp that handle and pull toward the robots along world -X by 100-120 mm; maximum travel is 130 mm. Release and leave it open and still for one simulation second. Do not close it again or remove it from its rails. The right arm observes only; the cabinet needs no hand support.",
        parameters={"outward_axis_world": (-1.0, 0.0, 0.0), "required_open_m": 0.10,
                    "max_travel_m": 0.13, "max_speed_m_s": 0.25},
    )


BUILDERS = {
    "L2-03": books,
    "L2-04": bowl,
    "L2-05": latch,
    "L2-06": jelly,
    "L2-07": drawer,
}
