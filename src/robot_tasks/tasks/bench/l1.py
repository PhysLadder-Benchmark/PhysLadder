"""Wide-clearance L1 props and one explicitly labelled flexible-stem proxy."""

import math

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, box, pose
from .geometry import cup_parts, cylinder, design, translated
from .spec import BenchSpec, Joint


def marker():
    parts = (
        cylinder("barrel", 0.015, 0.10, (0, 0, 0.05), (0.9, 0.7, 0.12)),
        cylinder("cap", 0.016, 0.025, (0, 0, 0.1125), BLUE),
    )
    a = Article(
        "marker", parts, pose(0.36, 0.42, 0.001), pose(0.36, 0.22, 0.004), 0.055
    )
    fixture = translated(
        cup_parts(bottom_radius=0.029, top_radius=0.036, height=0.065), (0.36, 0.22, 0)
    )
    return BenchSpec(
        design("L1-01", "马克笔入笔筒", [a], fixture, friction=0),
        facts="The capped marker is 0.125 m tall and at most 0.032 m wide. The fixed pen holder is 0.065 m tall with a wide opening. Keep the cap on. Place the lower marker end inside, then release.",
        parameters={
            "holder_xy": (0.36, 0.22),
            "rim": 0.065,
            "floor": 0.004,
            "radius": 0.029,
            "required_depth": 0.04,
        },
    )


def plug():
    # A keyed, low-force rectangular dummy connector. No electrical conductors/cable tension.
    parts = (
        box("grip", (0.038, 0.030, 0.042), (0, 0, 0.046), BLUE),
        box("tongue", (0.022, 0.014, 0.030), (0, 0, 0.015), GREY),
    )
    a = Article("plug", parts, pose(0.36, 0.42, 0.001), pose(0.36, 0.23, 0.004), 0.035)
    from ..l1.specs import rectangular_well

    fixture = rectangular_well((0.36, 0.23), (0.032, 0.024), 0.026, 0.004)
    return BenchSpec(
        design("L1-03", "低阻力插头插接", [a], fixture, friction=0),
        facts="This is an unpowered enlarged dummy connector, not a commercial electrical connector. Its rectangular tongue points downward initially, parallel to the upward-open socket axis. Grip the blue upper body. The slot has 0.005 m side clearance; leave the plug seated and released.",
        limitations="Rigid dummy connector; no cable, electrical continuity, retention spring or commercial connector fidelity.",
        parameters={
            "socket_xy": (0.36, 0.23),
            "opening": (0.032, 0.024),
            "tongue": (0.022, 0.014),
            "floor": 0.004,
            "rim": 0.030,
            "depth": 0.022,
        },
    )


def key():
    parts = (
        box("blade", (0.010, 0.018, 0.040), (0, 0, 0.020), GREY),
        box("bow", (0.023, 0.042, 0.032), (0, 0, 0.056), (0.95, 0.65, 0.08)),
    )
    # Support blade in a shallow open cradle; bow is exposed, never attached.
    from ..l1.specs import rectangular_well

    source = translated(
        rectangular_well((0, 0), (0.017, 0.026), 0.024, 0.004),
        (0.36, 0.42, 0),
        "source_",
    )
    target = translated(
        rectangular_well((0, 0), (0.018, 0.028), 0.032, 0.004), (0.36, 0.23, 0), "lock_"
    )
    a = Article("key", parts, pose(0.36, 0.42, 0.005), pose(0.36, 0.23, 0.004), 0.028)
    return BenchSpec(
        design("L1-05", "钥匙插锁（柄部外露）", [a], source + target, friction=0),
        facts="The yellow key bow protrudes above its grey source cradle. Lift it out, align the flat blade with the fixed target keyway and insert without turning. The bow may remain held at completion.",
        limitations="Enlarged flat dummy key/keyway; no lock tumblers or unlocking mechanism.",
        parameters={
            "socket_xy": (0.36, 0.23),
            "opening": (0.018, 0.028),
            "tongue": (0.010, 0.018),
            "floor": 0.004,
            "rim": 0.036,
            "depth": 0.028,
        },
    )


def mug():
    # Handle lies in cup-local XZ; hole width 30 mm, height 32 mm.
    parts = cup_parts(
        bottom_radius=0.020, top_radius=0.024, height=0.065, color=RED
    ) + (
        box("handle_top", (0.045, 0.016, 0.016), (0.044, 0, 0.061), RED),
        box("handle_bottom", (0.045, 0.016, 0.016), (0.044, 0, 0.013), RED),
        box("handle_side", (0.016, 0.016, 0.048), (0.065, 0, 0.037), RED),
    )
    fixtures = (
        box("rack_foot", (0.10, 0.10, 0.012), (0.37, 0.16, 0.006)),
        cylinder("rack_post", 0.008, 0.22, (0.37, 0.16, 0.122), GREY),
        cylinder("hook", 0.004, 0.09, (0.37, 0.205, 0.20), GREY, (math.pi / 2, 0, 0)),
        cylinder("hook_tip", 0.004, 0.022, (0.37, 0.247, 0.209), GREY),
    )
    a = Article("mug", parts, pose(0.35, 0.44, 0.001), pose(0.328, 0.21, 0.147), 0.060)
    return BenchSpec(
        design("L1-07", "塑料杯挂上杯架", [a], fixtures),
        facts="The small empty red cup has a rigid rectangular loop handle. Its body is about 0.054 m wide. Hang it through the HANDLE, not the mouth, on the single upward-ended rack hook. The handle opening is about 0.030 by 0.032 m and the hook is 0.008 m thick.",
        parameters={
            "hook_start": (0.37, 0.17, 0.20),
            "hook_end": (0.37, 0.244, 0.20),
            "handle_centre": (0.042, 0, 0.037),
            "handle_half": (0.015, 0.016),
        },
    )


def cups():
    lower = Article(
        "lower_cup",
        cup_parts(bottom_radius=0.021, top_radius=0.033, height=0.060),
        pose(0.36, 0.23, 0.001),
        pose(0.36, 0.23, 0),
        0.11,
    )
    upper = Article(
        "upper_cup",
        cup_parts(
            bottom_radius=0.013, top_radius=0.022, height=0.073, wall=0.003, color=RED
        ),
        pose(0.36, 0.43, 0.001),
        pose(0.36, 0.23, 0.004),
        0.045,
    )
    return BenchSpec(
        design("L1-08", "两个杯子宽松套叠", [lower, upper]),
        facts="Put the smaller red cup into the larger blue cup. Both cups start upright and empty. The lower blue cup is movable, not glued down. Grip high on the red cup so the fingers can clear the blue rim. Release once supported; no vertical-axis yaw target is required.",
    )


def flower():
    # Seven rigid segments connected by passive bending springs: a rod proxy, not plant tissue.
    count, length, radius = 7, 0.028, 0.008
    articles, joints = [], []
    for i in range(count):
        xyz = (0.24 + i * length, 0.41, 0.051)
        articles.append(
            Article(
                f"stem_{i}",
                (cylinder("stem", radius, length, color=GREEN),),
                pose(*xyz, pitch=math.pi / 2),
                pose(*xyz, pitch=math.pi / 2),
                0.005,
            )
        )
        if i:
            joints.append(
                Joint(
                    f"bend_{i}",
                    f"stem_{i - 1}",
                    f"stem_{i}",
                    (xyz[0] - length / 2, xyz[1], xyz[2]),
                    {a: (-85, 85, 0.00005, 0.0005) for a in ("rotX", "rotY", "rotZ")},
                )
            )
    head_xyz = (0.24 + count * length, 0.41, 0.051)
    articles.append(
        Article(
            "flower_head",
            (cylinder("head", 0.050, 0.040, color=(0.9, 0.2, 0.55)),),
            pose(*head_xyz, pitch=math.pi / 2),
            pose(*head_xyz),
            0.025,
        )
    )
    joints.append(
        Joint(
            "head_joint",
            f"stem_{count - 1}",
            "flower_head",
            (head_xyz[0] - length / 2, head_xyz[1], head_xyz[2]),
            {a: (-85, 85, 0.00005, 0.0005) for a in ("rotX", "rotY", "rotZ")},
        )
    )
    vase = translated(
        cup_parts(bottom_radius=0.030, top_radius=0.040, height=0.18, wall=0.005),
        (0.36, 0.20, 0),
    )
    return BenchSpec(
        design("L1-09", "软茎花材插瓶（柔性杆代理）", articles, vase),
        joints=tuple(joints),
        mechanism="passive_jointed_rod_proxy",
        limitations="Segmented bending rod and heavy head; not calibrated plant tissue, no petal bruising or stem fracture. Only the initial-grasp version is assigned L1.",
        facts="The flower is a flexible seven-segment stem with a heavier pink head. Far-end support lets it sag. Choose a supportive grasp leaving the green free tip clear, and put that tip into the wide blue vase. The spring-rod material is a simulation proxy, not a real flower species.",
        parameters={
            "vase_xy": (0.36, 0.20),
            "radius": 0.030,
            "rim": 0.18,
            "depth": 0.025,
        },
    )


BUILDERS = {
    "L1-01": marker,
    "L1-03": plug,
    "L1-05": key,
    "L1-07": mug,
    "L1-08": cups,
    "L1-09": flower,
}
