"""Relative screw contact, tool-mediated pushing and bimanual sweeping."""

import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, Part, box, pose
from ..screw.geometry import helix_pieces
from ..screw.spec import SPEC as SCREW
from .geometry import cup_parts, cylinder, design, tray_parts
from .spec import BenchSpec


def bottle():
    thread = replace(
        SCREW,
        core_radius=0.012,
        crest_radius=0.0145,
        radial_clearance=0.001,
        pitch=0.006,
        thread_start=0.110,
        thread_turns=2.0,
        socket_thread_start=-0.017,
        socket_thread_turns=2.0,
        thread_root_width=0.0026,
        thread_crest_width=0.0014,
        segments_per_turn=96,
    )
    body = [
        cylinder("bottle_body", 0.026, 0.10, (0, 0, 0.05), (0.25, 0.50, 0.12)),
        cylinder("neck", 0.012, 0.034, (0, 0, 0.117), (0.65, 0.73, 0.3)),
    ]
    cap = list(
        cup_parts(
            "cap",
            bottom_radius=0.0175,
            top_radius=0.0175,
            height=0.030,
            wall=0.004,
            floor=0.004,
            color=BLUE,
        )
    )
    # Invert the open cup: cap mouth at z=-.015 and closed roof at z=.015.
    cap = [
        replace(
            p,
            position=(p.position[0], p.position[1], 0.015 - p.position[2]),
            rotation=(math.pi, 0, 0),
        )
        for p in cap
    ]
    cap.append(
        box(
            "rotation_mark",
            (0.013, 0.003, 0.0004),
            (0.005, 0, 0.0152),
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
        Article("cap", tuple(cap), pose(0.36, 0, 0.131), pose(0.36, 0, 0.131), 0.025),
    ]
    return BenchSpec(
        design("L3-03", "东方树叶外形瓶盖旋拧（工程螺纹代理）", articles),
        arms=("left", "right"),
        mechanism="free_bottle_and_explicit_helical_contact",
        limitations="Generic bottle-shaped engineering proxy; not an official Oriental Leaf CAD or measured thread. No tamper band, seal break, liquid or plastic damage model. Cap starts engaged, not pre-loosened.",
        facts="Hold the green bottle body with one arm and unscrew the blue cap with the other. The body is 0.052 m wide; the cap is about 0.043 m wide. The right-hand thread has a coarse simulation pitch of 0.006 m. Counterclockwise rotation viewed from above raises the cap. Allow axial rise, regrasp as needed, and verify complete disengagement before pulling free. Release both parts on the table. This is a disclosed engineering thread proxy, not measured Oriental Leaf geometry.",
        parameters={
            "pitch": 0.006,
            "neck_top": 0.134,
            "cap_half": 0.015,
            "initial_cap_z": 0.130,
            "minimum_turn": 2 * math.pi,
            "rise": 0.014,
            "thread_friction": 0.05,
            "thread_contact_offset_m": 0.00015,
            "engaged_cap_z_max": 0.142,
        },
    )


def pushing():
    parts = (
        box("crossbar", (0.10, 0.030, 0.026), (0, 0.025, 0), RED),
        box("stem", (0.030, 0.070, 0.026), (0, -0.025, 0), RED),
    )
    tool = (
        box("handle", (0.09, 0.025, 0.026), (-0.02, 0, 0), BLUE),
        box("head", (0.020, 0.050, 0.030), (0.035, 0, 0), GREEN),
    )
    articles = [
        Article(
            "t_block",
            parts,
            pose(0.39, 0.43, 0.014, yaw=0.65),
            pose(0.39, 0.22, 0.013),
            0.10,
        ),
        Article("tool", tool, pose(0.23, 0.43, 0.016), pose(0.23, 0.43, 0.016), 0.045),
    ]
    # Visual flat footprint only; it never acts as a funnel or fitted recess.
    target = tuple(
        replace(
            p,
            name="outline_" + p.name,
            size=(p.size[0], p.size[1], 0.0004),
            position=(0.39 + p.position[0], 0.22 + p.position[1], 0.0002),
            collision=False,
            color=(0.1, 0.6, 0.25),
        )
        for p in parts
    )
    return BenchSpec(
        design("L3-14", "持工具推 T 形块定姿", articles, target),
        facts="Grasp the blue tool handle and use its green head to push the red T block onto the flat green T footprint. Do not grasp or lift the T block directly. Adjust push points and direction as the block turns. Position tolerance is 0.012 m and yaw tolerance 12 degrees. Keep the tool grasped at completion and withdraw its head from the block.",
        limitations="Rigid frictional planar pushing. L3 interpretation requires analysis of contact changes; simple geometric success alone does not prove force-adaptation ability.",
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


BUILDERS = {"L3-03": bottle, "L3-14": pushing, "L3-15": sweeping}
