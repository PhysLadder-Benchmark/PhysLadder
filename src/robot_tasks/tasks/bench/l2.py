"""Passive support, natural sliding/latch resistance and a declared elastic proxy."""

import itertools

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, box, pose
from .geometry import cup_parts, design, floor_target, translated, tray_parts
from .spec import BenchSpec, Joint


def books():
    articles = []
    for name, xyz, color in (
        ("lower_book", (0.37, 0, 0.045), BLUE),
        ("middle_book", (0.41, 0, 0.070), RED),
        ("upper_book", (0.37, 0, 0.095), GREEN),
    ):
        articles.append(
            Article(
                name,
                (box("cover", (0.14, 0.056, 0.025), color=color),),
                pose(*xyz),
                pose(*xyz),
                0.09,
            )
        )
    supports = tuple(
        box(f"support_{i}", (0.022, 0.07, 0.032), (x, 0, 0.016))
        for i, x in enumerate((0.32, 0.42))
    )
    return BenchSpec(
        design("L2-03", "三本书中层抽取与上层支撑", articles, supports),
        arms=("left", "right"),
        facts="Three small rigid books rest on two low supports. The red middle book protrudes toward the front (+X). Support the green top book with one arm while the other pulls out the red book, then place both on the table. Book width is 0.056 m, within the gripper range.",
        parameters={"extract_distance": 0.14},
    )


def bowl():
    bowl_parts = cup_parts(
        bottom_radius=0.045, top_radius=0.064, height=0.055, color=BLUE
    ) + (
        box("handle_left", (0.030, 0.036, 0.018), (0, 0.077, 0.040), BLUE),
        box("handle_right", (0.030, 0.036, 0.018), (0, -0.077, 0.040), BLUE),
    )
    articles = [
        Article("bowl", bowl_parts, pose(0.37, 0.21, 0.001), pose(0.40, 0, 0.13), 0.10)
    ]
    # Passive blocks rest above a sliding gate. Only the feeder's gate is scripted.
    for i, (x, y) in enumerate(itertools.product((-0.026, 0, 0.026), (-0.015, 0.015))):
        articles.append(
            Article(
                f"load_{i}",
                (box("block", (0.016, 0.016, 0.016), color=RED),),
                pose(0.40 + x, y, 0.322),
                pose(0.40 + x, y, 0.322),
                0.020,
            )
        )
    articles.append(
        Article(
            "feed_gate",
            (box("plate", (0.10, 0.07, 0.004), color=GREY),),
            pose(0.40, 0, 0.311),
            pose(0.40, 0, 0.311),
            0.10,
        )
    )
    joint = Joint(
        "feed_slide", "", "feed_gate", (0.40, 0, 0.311), {"transX": (0, 0.15, 300, 25)}
    )
    # Retaining walls keep the load above the receiver as the bottom gate
    # withdraws. Without them friction carries every block away on the plate.
    hopper = (
        box("feed_left", (0.004, 0.064, 0.050), (0.358, 0, 0.338), GREY),
        box("feed_right", (0.004, 0.064, 0.050), (0.442, 0, 0.338), GREY),
        box("feed_back", (0.080, 0.004, 0.050), (0.40, -0.030, 0.338), GREY),
        box("feed_front", (0.080, 0.004, 0.050), (0.40, 0.030, 0.338), GREY),
    )
    return BenchSpec(
        design("L2-04", "持碗动态固体负载保持", articles, hopper),
        arms=("left", "right"),
        joints=(joint,),
        mechanism="rigid_solids_and_environment_feeder",
        facts="Lift the blue two-handled bowl to the feed position below the grey gate (bowl bottom approximately 0.13 m above the table). Hold both handles and keep the bowl level. Once held there, a standardized feeder slowly withdraws its gate over three simulation seconds and drops six red blocks. Hold the loaded bowl steadily for two seconds, then place it on the table and release without spilling; no water is simulated.",
        parameters={"feed_xy": (0.40, 0), "feed_z": 0.13, "count": 6},
    )


def latch():
    # Hinge along world Y at x=.29. Spring slider on the front of the lid catches a fixed ledge.
    center = (0.37, 0)
    walls = tray_parts("box", width=0.16, depth=0.12, wall_height=0.05) + (
        box("front_wall", (0.004, 0.12, 0.05), (0.08, 0, 0.025), BLUE),
        box("catch", (0.012, 0.050, 0.006), (0.084, 0, 0.047), GREY),
    )
    fixtures = translated(walls, (*center, 0))
    lid_parts = (
        box("panel", (0.17, 0.13, 0.006), color=BLUE),
        box("grip", (0.028, 0.030, 0.022), (0.048, 0, 0.014), GREEN),
    )
    latch_parts = (
        box("button", (0.014, 0.040, 0.032), (0, 0, 0), RED),
        box("tooth", (0.012, 0.032, 0.006), (-0.009, 0, -0.012), RED),
    )
    articles = [
        Article("lid", lid_parts, pose(0.37, 0, 0.058), pose(0.37, 0, 0.058), 0.06),
        Article(
            "latch", latch_parts, pose(0.466, 0, 0.051), pose(0.466, 0, 0.051), 0.008
        ),
        Article(
            "eraser",
            (box("body", (0.028, 0.040, 0.018), color=(0.95, 0.75, 0.18)),),
            pose(0.37, 0, 0.014),
            pose(0.50, 0.20, 0.009),
            0.025,
        ),
    ]
    joints = (
        Joint("lid_hinge", "", "lid", (0.285, 0, 0.055), {"rotY": (-115, 0, 0, 0.005)}),
        Joint(
            "latch_spring",
            "lid",
            "latch",
            (0.466, 0, 0.051),
            {"transX": (0, 0.020, 60, 0.1)},
        ),
    )
    return BenchSpec(
        design("L2-05", "弹性卡扣翻盖盒开合", articles, fixtures),
        arms=("left", "right"),
        joints=joints,
        mechanism="hinge_and_passive_spring_latch",
        limitations="Enlarged spring-slider latch proxy, not a calibrated moulded flexure; no fracture model.",
        facts="The blue box is fixed. Its green lid handle is graspable. The red front latch has a spring-return tooth engaged below a fixed ledge. Pull the red latch outward (+X) about 0.015 m to clear the catch while lifting the lid with the other hand. Remove the yellow eraser and place it on the table. Hold the latch out while closing the lid, then release the latch to re-engage. Do not force the tooth through the catch.",
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
    # Actual tray/rail contacts produce sliding friction; the joint only constrains the track.
    tray = tray_parts("drawer", width=0.14, depth=0.12, wall_height=0.04) + (
        box("front", (0.008, 0.13, 0.052), (0.074, 0, 0.026), RED),
        box("handle", (0.024, 0.040, 0.024), (0.096, 0, 0.028), GREEN),
    )
    a = Article("drawer", tray, pose(0.34, 0.30, 0.014), pose(0.34, 0.30, 0.014), 0.13)
    fixtures = (
        box("rail_bottom", (0.32, 0.15, 0.012), (0.40, 0.30, 0.006), GREY),
        box("cabinet_left", (0.18, 0.008, 0.066), (0.32, 0.225, 0.033), BLUE),
        box("cabinet_right", (0.18, 0.008, 0.066), (0.32, 0.375, 0.033), BLUE),
        box("cabinet_back", (0.008, 0.15, 0.066), (0.23, 0.30, 0.033), BLUE),
    )
    joint = Joint(
        "drawer_slide",
        "",
        "drawer",
        (0.34, 0.30, 0.014),
        {"transX": (0, 0.13, 0, 0.15), "transZ": (-0.004, 0.002, 0, 0)},
    )
    return BenchSpec(
        design("L2-07", "化妆盒抽屉平稳推拉", [a], fixtures, friction=0.25),
        joints=(joint,),
        mechanism="prismatic_rail_and_contact_friction",
        facts="The blue cabinet is fixed. Grasp the green handle, pull the drawer toward world +X by at least 0.10 m, then push it closed and release. The track limits travel to 0.13 m. Use gentle motions near either stop; do not lift or twist the drawer.",
    )


BUILDERS = {
    "L2-03": books,
    "L2-04": bowl,
    "L2-05": latch,
    "L2-06": jelly,
    "L2-07": drawer,
}
