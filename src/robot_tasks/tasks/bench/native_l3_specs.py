"""Delivered rigid props in the laboratory frame; no pregrasp or action script."""

import math
from dataclasses import replace

from ..l1.specs import Article, box, pose
from .geometry import design
from .spec import BenchSpec

TABLE_Z = 0.78


def _article(name, asset, initial, mass, bounds):
    return Article(
        name, (), initial, initial, mass, asset="tuojing:" + asset, asset_bounds=bounds
    )


def bottle():
    base = pose(0.19565, 1.30, TABLE_Z)
    # Source cap geometry is expressed in the bottle frame, not at cap centre.
    # Three-start photo-estimated RH threads have 9 mm lead per revolution.
    # +7.5 mm / +300 degrees leaves the last partial engagement, no seal preload.
    lift, turn = 0.0075, 5 * math.pi / 3
    cap = pose(base[0], base[1], base[2] + lift, yaw=turn)
    articles = [
        _article(
            "bottle",
            "oriental_leaf_335ml",
            base,
            0.358,
            ((-0.031, -0.023, 0.001), (0.031, 0.023, 0.1865)),
        ),
        _article(
            "cap",
            "oriental_leaf_335ml",
            cap,
            0.0035,
            ((-0.02078, -0.02078, 0.1742), (0.02078, 0.02078, 0.1900)),
        ),
    ]
    return BenchSpec(
        replace(
            design("L3-03", "双臂扶东方树叶瓶旋下松盖", articles),
            version="l3-03-centred-bimanual-loose-cap-v5",
            release=False,
            provenance="Delivered Oriental Leaf 335 ml body/cap, pre-loosened initial state",
        ),
        arms=("left", "right"),
        mechanism="native_bimanual_loose_cap_contact",
        limitations="Photo-estimated native interrupted threads, not measured opening torque. Body is free and must be supported by the other arm; cap pre-loosened 7.5 mm/300 degrees, no tamper-band breaking or seal preload. Rigid visual liquid, no slosh. This simplified cap-removal task does not measure breaking a tight seal.",
        facts="The Oriental Leaf 335 ml bottle stands freely between both arms. LEFT steadies the lower straight section of the bottle body, preferably keeping its base on the table; RIGHT grasps the transparent cap sidewall (about 42 mm diameter, 16 mm tall). Both arms manipulate; neither is observation-only. The body is about 62 x 46 mm wide: grip the narrower parallel faces below the shoulder and leave vertical clearance for the cap hand. The cap is already loosened, with only the last partial thread engagement. Turn counterclockwise viewed from above and allow it to rise; inspect whether it is already free before pulling away. No prescribed turn count or minimum measured torque. Keep the LEFT body grasp while the RIGHT turns and removes the cap; do not pinch the neck with the cap hand. Once completely detached, place the cap anywhere clear on the table and release it. The bottle may remain supported by LEFT, or stand upright on the table after release. No exact parking point or timed wait is required.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.0,
            "body_arm": "left",
            "cap_arm": "right",
            "cap_bottom_z": 0.1742,
            "cap_top_z": 0.1900,
            "cap_radius_m": 0.02078,
            "neck_radius_m": 0.019,
            "neck_top_z": 0.1865,
            "thread_lead_m": 0.009,
            "initial_cap_lift_m": lift,
            "native_instances": [
                {
                    "name": "bottle_assembly",
                    "asset": "oriental_leaf_335ml",
                    "pose": base,
                    "tracked": {"bottle": "Body", "cap": "Cap"},
                    "initial_body_transforms": {"Cap": pose(0, 0, lift, yaw=turn)},
                }
            ],
        },
    )


def wiping():
    board = pose(0.25, 1.29, TABLE_Z, yaw=math.pi)
    eraser = pose(
        0.505, 1.30, TABLE_Z + 0.025 + 0.0184 + 0.001, roll=math.pi, yaw=math.pi / 2
    )
    a = _article(
        "eraser",
        "board_eraser",
        eraser,
        0.020,
        ((-0.05, -0.025, 0), (0.05, 0.025, 0.0184)),
    )
    return BenchSpec(
        replace(
            design(
                "L3-11",
                "抓取板擦擦除绿板字迹（清洁代理）",
                [a],
                (
                    box(
                        "eraser_start_rest",
                        (0.030, 0.060, 0.025),
                        (0.505, 1.30, TABLE_Z + 0.0125),
                        (0.32, 0.33, 0.35),
                    ),
                ),
            ),
            version="l3-11-inverted-gpt6-right-arm-v2",
            release=False,
            provenance="2026-10-08 native board and eraser; contact-driven visual ink proxy",
        ),
        arms=("right",),
        mechanism="native_contact_wiping",
        limitations="Rigid eraser, fixed green board and discrete equal-area ink cells. Contact plus measured sliding removes cells; 25 mm passive starting rest for finger clearance; no real ink chemistry, moisture, pad compliance or calibrated pressure-cleaning law. This simplified proxy does not establish real L3 cleaning competence.",
        facts="The green board is fixed flat on the table (410 x 280 mm). The loose eraser is 100 x 50 x 18 mm and starts pad-down on a low passive rest beside it. Grip the upper plastic side edges across the 50 mm width, keeping finger tips above the cleaning face; the long 100 mm direction cannot fit the gripper. Its cleaning face is opposite its plastic back. The board and BLACK GPT6 lettering are rotated 180 degrees in the tabletop plane, so the text is upside-down from the shared overview image. The eraser starts on the board's image-right. Wipe with the RIGHT hand; LEFT observes only. Clean at least 90% of the initial visible writing. No timed hold, final eraser placement or specific force value is required. Hovering and occlusion cannot erase marks.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.0,
            "required_clean_fraction": 0.90,
            "board_pose": board,
            "board_surface_z": 0.010,
            "native_instances": [
                {
                    "name": "board",
                    "asset": "green_board_20261008",
                    "pose": board,
                    "static": [""],
                    "tracked": {},
                },
                {
                    "name": "eraser",
                    "asset": "board_eraser",
                    "pose": eraser,
                    "tracked": {"eraser": ""},
                },
            ],
        },
    )


def sweeping():
    # Native thin handles cannot be pinched through their 1.2/8 mm thickness.
    # Passive starting rests expose the 28/32 mm handle WIDTH; they do not move
    # or attach to the tool and are not part of the goal region.
    pan = pose(0.0913, 1.31, TABLE_Z, yaw=-math.pi / 2)
    broom = pose(0.4813, 1.31, TABLE_Z + 0.026, yaw=math.pi / 2)
    articles = [
        _article(
            "dustpan", "dustpan", pan, 0.0232, ((-0.07, -0.14, 0), (0.07, 0.04, 0.018))
        ),
        _article(
            "broom",
            "hand_brush",
            broom,
            0.0365,
            ((-0.07, -0.105, 0), (0.07, 0.0702, 0.008)),
        ),
    ]
    instances = [
        {
            "name": a.name,
            "asset": a.asset.removeprefix("tuojing:"),
            "pose": a.initial,
            "tracked": {a.name: ""},
        }
        for a in articles
    ]
    instances[0]["static"] = [""]
    for i, (x, y) in enumerate(((0.2913, 1.27), (0.2513, 1.32))):
        a = _article(
            f"block_{i}",
            "blue_square",
            pose(x, y, TABLE_Z + 0.001),
            0.003,
            ((-0.01, -0.01, 0), (0.01, 0.01, 0.01)),
        )
        articles.append(a)
        instances.append(
            {
                "name": a.name,
                "asset": "blue_square",
                "pose": a.initial,
                "tracked": {a.name: ""},
            }
        )
    rests = (
        box(
            "brush_start_rest",
            (0.06, 0.10, 0.025),
            (0.4713, 1.31, TABLE_Z + 0.0125),
            (0.32, 0.33, 0.35),
        ),
    )
    return BenchSpec(
        replace(
            design("L3-15", "单臂持手刷扫方块入固定簸箕", articles, rests),
            version="l3-15-right-brush-two-blocks-v4",
            release=False,
            provenance="2026-10-08 native hand brush, dustpan and two blue square blocks",
        ),
        arms=("right",),
        mechanism="native_rigid_brush_sweeping",
        limitations="Delivered geometry and mass; bristles are rigid capsule bundles, not elastic fibers. A passive 25 mm brush starting rest exposes its handle to an R5 side pinch. The dustpan is fixed at tabletop height. Friction is estimated; no fine dust or material damage model.",
        facts="The dustpan is FIXED with its thin entry lip on the tabletop; do not grasp or move it. From the shared overview image, the pan is on the LEFT with its mouth toward the RIGHT; the brush starts on the RIGHT, with TWO blocks between them. RIGHT grips the loose small hand brush; LEFT observes only. The two BLUE blocks are 20 x 20 x 10 mm. The brush rests on a low passive support. Pinch its handle ACROSS the roughly 28 mm width, not the 8 mm thickness. Bristles project forward in the handle plane. Sweep BOTH blocks across the open straight lip into the pan basket. Direct gripper transport of blocks does not count. No final brush release, exact yaw or timed wait is required.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.0,
            "count": 2,
            "required": 2,
            "pan_floor_z": 0.0012,
            "native_instances": instances,
        },
    )
