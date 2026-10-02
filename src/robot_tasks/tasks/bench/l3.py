"""Relative screw contact, direct gripper pushing and bimanual sweeping."""

import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, Part, box, pose
from ..screw.geometry import helix_pieces
from ..screw.spec import SPEC as SCREW
from .geometry import cup_parts, cylinder, design, tray_parts
from .spec import BenchSpec
from .push_drawer import build as push_drawer
from .push_t import build as pushing


def bottle():
    # Keep jaw width modest; separate the body and cap grasp regions vertically.
    body_width, body_height = 0.050, 0.180
    neck_top, cap_z, cap_height = 0.234, 0.230, 0.040
    cap_half = cap_height / 2
    thread = replace(
        SCREW,
        core_radius=0.012,
        crest_radius=0.0145,
        radial_clearance=0.001,
        pitch=0.006,
        thread_start=neck_top - 0.024,
        thread_turns=2.0,
        socket_thread_start=-0.017,
        socket_thread_turns=2.0,
        thread_root_width=0.0026,
        thread_crest_width=0.0014,
        segments_per_turn=96,
    )
    body = [
        box("bottle_body", (body_width, body_width, body_height),
            (0, 0, body_height / 2), (0.25, 0.50, 0.12)),
        cylinder("neck", 0.012, neck_top - body_height,
                 (0, 0, (body_height + neck_top) / 2), (0.65, 0.73, 0.3)),
    ]
    cap = list(
        cup_parts(
            "cap",
            bottom_radius=0.0175,
            top_radius=0.0175,
            height=cap_height,
            wall=0.004,
            floor=0.004,
            color=BLUE,
        )
    )
    # Invert the open cup around its centre; the roof must remain above the neck.
    cap = [
        replace(
            p,
            position=(p.position[0], p.position[1], cap_half - p.position[2]),
            rotation=(math.pi, 0, 0),
        )
        for p in cap
    ]
    cap.append(
        box(
            "rotation_mark",
            (0.013, 0.003, 0.0004),
            (0.005, 0, cap_half + 0.0002),
            (0.95, 0.75, 0.10),
            collision=False,
        )
    )
    # Mesh wall points live in their own frame: flipping both geometry and translation is intentional.
    for female, dest in ((False, body), (True, cap)):
        for i, (vertices, faces) in enumerate(helix_pieces(female=female, spec=thread)):
            dest.append(
                Part(
                    f"thread_{i}",
                    (0, 0, 0),
                    shape="mesh",
                    color=GREY,
                    vertices=tuple(tuple(float(v) for v in row) for row in vertices),
                    faces=tuple(tuple(int(v) for v in row) for row in faces),
                )
            )
    articles = [
        Article(
            "bottle", tuple(body), pose(0.36, 0, 0.001), pose(0.36, 0, 0.001), 0.20
        ),
        Article("cap", tuple(cap), pose(0.36, 0, cap_z + 0.001),
                pose(0.36, 0, cap_z + 0.001), 0.025),
    ]
    return BenchSpec(
        replace(design("L3-03", "双臂抓持方瓶旋盖（工程螺纹代理）", articles),
                version="l3-03-square-bottle-v2"),
        arms=("left", "right"),
        mechanism="free_bottle_and_explicit_helical_contact",
        limitations="Tall square-body engineering proxy; not an official Oriental Leaf CAD or measured thread. No tamper band, seal break, liquid or plastic damage model. Cap starts engaged, not pre-loosened. Wrist camera housings/brackets remain visual-only with provisional mounts; rendered clearance is not physical collision enforcement.",
        facts="Keep the green square bottle standing on the table. Stabilize its parallel 0.050 m wide faces at the lower/middle part of the 0.180 m tall body with the left hand. Use the right hand on the blue cylindrical cap, about 0.043 m wide and 0.040 m tall, above the round neck. Keep the grasp regions separated vertically, and inspect clearance for fingers, wrists and camera brackets during rotation and regrasping. The right-hand thread has a coarse simulation pitch of 0.006 m. Counterclockwise rotation viewed from above raises the cap. Allow axial rise, regrasp as needed, and verify complete disengagement before pulling free. Release both parts on the table. This is an engineering thread proxy, not measured Oriental Leaf geometry.",
        parameters={
            "body_width": body_width,
            "body_height": body_height,
            "pitch": 0.006,
            "neck_top": neck_top,
            "cap_half": cap_half,
            "initial_cap_z": cap_z,
            "minimum_turn": 2 * math.pi,
            "rise": neck_top + cap_half + 0.004 - cap_z,
            "thread_friction": 0.05,
            "thread_contact_offset_m": 0.00015,
            "engaged_cap_z_max": cap_z + 0.012,
        },
    )


def sweeping():
    pan = tray_parts("pan", width=0.14, depth=0.11, wall_height=0.025, floor=0.0015) + (
        box("handle", (0.025, 0.10, 0.025), (0, -0.095, 0.03), BLUE),
    )
    broom = (
        box("handle", (0.025, 0.13, 0.025), (0, 0, 0.025), GREEN),
        box("blade", (0.09, 0.025, 0.030), (0, 0.075, 0.015), RED),
    )
    articles = [
        Article("dustpan", pan, pose(0.39, -0.19, 0.001), pose(0.39, 0, 0.001), 0.085),
        Article(
            "broom", broom, pose(0.25, 0.23, 0.001), pose(0.25, 0.23, 0.001), 0.055
        ),
    ]
    for i, (x, y) in enumerate(
        ((0.36, 0.09), (0.40, 0.10), (0.44, 0.08), (0.38, 0.14), (0.43, 0.15))
    ):
        articles.append(
            Article(
                f"block_{i}",
                (box("cube", (0.014, 0.014, 0.014), color=(0.9, 0.7, 0.10)),),
                pose(x, y, 0.008),
                pose(x, y, 0.008),
                0.010,
            )
        )
    return BenchSpec(
        design("L3-15", "双臂扫块入簸箕", articles),
        arms=("left", "right"),
        facts="Use the left arm to grasp the green broom handle and the right arm to hold the blue dustpan handle. Its open front faces world +Y. Sweep the five yellow blocks through that opening into the pan; keep the lip near the table and adapt to the blocks. Do not pick up blocks with grippers. All five should end inside, with no blocks dropped from the table. The broom head is a rigid blade proxy.",
        limitations="Rigid broom blade and blocks; no compliant bristles or fine dust model.",
        parameters={"count": 5, "required": 5},
    )


BUILDERS = {"L3-03": bottle, "L3-14": pushing, "L3-15": sweeping, "L3-16": push_drawer}
