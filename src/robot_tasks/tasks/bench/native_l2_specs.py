"""L20 task layouts and goals in the delivered laboratory's metre/world frame."""

import itertools
import math
from dataclasses import replace

from ..l1.specs import BLUE, GREY, Article, box, pose
from .geometry import design, floor_target
from .spec import BenchSpec

TABLE_Z = 0.78

BOWL_INNER = (
    (0.004, 0.032),
    (0.014, 0.044),
    (0.030, 0.056),
    (0.050, 0.066),
    (0.054, 0.067),
)


def bowl_clearance(point, radius=0.004, tolerance=0.0003):
    """Sphere containment against the delivered 32-sided, piecewise inner wall."""
    x, y, z = point
    if z - radius < 0.006 - tolerance or z + radius > 0.054 + tolerance:
        return False
    cosine = math.cos(math.pi / 32)
    for (z0, r0), (z1, r1) in itertools.pairwise(BOWL_INNER):
        slope = (r1 - r0) / (z1 - z0)
        boundary = (r0 + slope * (z - z0)) * cosine
        margin = radius * math.sqrt(1 + (slope * cosine) ** 2)
        for i in range(32):
            a = (i + 0.5) * 2 * math.pi / 32
            if x * math.cos(a) + y * math.sin(a) + margin > boundary + tolerance:
                return False
    return True


def bowl():
    source, target = (0.19, 1.25, TABLE_Z + 0.001), (0.19, 1.13, TABLE_Z + 0.001)
    handles = tuple(
        box(f"grip_{i}", (0.040, 0.045, 0.020), (sign * 0.084, 0.0, 0.030), BLUE)
        for i, sign in enumerate((-1, 1))
    )
    a = Article(
        "bowl",
        (),
        pose(*source),
        pose(*target),
        0.136,
        asset="tuojing:open_bowl",
        asset_bounds=((-0.104, -0.070, 0.0), (0.104, 0.070, 0.054)),
    )
    articles = [a]
    instances = [
        {
            "name": "bowl",
            "asset": "open_bowl",
            "pose": a.initial,
            "tracked": {"bowl": ""},
            "adapters": {"": handles},
            "mass_additions": {"": 0.016},
        }
    ]
    radius, spacing, mass = 0.004, 0.0084, 0.00030895229571687653
    for layer in range(6):
        z = 0.0103 + layer * spacing * math.sqrt(2 / 3)
        offset = (spacing / 2, spacing * math.sqrt(3) / 6) if layer % 2 else (0.0, 0.0)
        for i, j in itertools.product(range(-8, 9), repeat=2):
            x, y = (
                spacing * (i + j / 2) + offset[0],
                spacing * math.sqrt(3) / 2 * j + offset[1],
            )
            if not bowl_clearance((x, y, z), radius, tolerance=-0.0003):
                continue
            name = f"load_{len(articles) - 1}"
            initial, goal = (
                pose(source[0] + x, source[1] + y, source[2] + z),
                pose(target[0] + x, target[1] + y, target[2] + z),
            )
            articles.append(
                Article(
                    name,
                    (),
                    initial,
                    goal,
                    mass,
                    asset="tuojing:pearl_8mm",
                    asset_bounds=(
                        (-radius, -radius, -0.003968627),
                        (radius, radius, 0.003968627),
                    ),
                )
            )
            instances.append(
                {
                    "name": name,
                    "asset": "pearl_8mm",
                    "pose": initial,
                    "tracked": {name: ""},
                }
            )
    count = len(articles) - 1
    patch = replace(
        floor_target(target[:2], (0.20, 0.20)),
        position=(target[0], target[1], TABLE_Z + 0.0002),
    )
    return BenchSpec(
        replace(
            design("L2-04", "满载小珍珠的碗搬运", articles, [patch]),
            version="l2-04-native-bowl-pearls-v1",
            provenance="Native open bowl and 8 mm drilled plastic pearls; two explicit R5 grasp tabs",
        ),
        arms=("left", "right"),
        mechanism="native_loaded_bowl_transport",
        limitations="Native bowl compound colliders and pearl visuals/dimensions/masses. Pearl bore collision is replaced by an outer sphere for transport. Two blue grasp tabs add 16 g; bead friction explicitly 0.03. No fluid or damage model.",
        facts=f"A cream-coloured open bowl is preloaded with {count} separate 8 mm pearls. They move freely under gravity and contact; their contact friction is explicitly 0.03 in this task. The bowl is 140 mm across and 54 mm high, and has TWO blue R5 grasp tabs attached outside its sides. These are declared adapters, not native bowl handles. Use the left hand on the -X tab and right on the +X tab; pinch the 45 mm dimension along world Y and leave the opening clear. Lift together, move to the green patch, lower, release both hands and leave all pearls in the upright bowl. No feeder or second loading phase is used.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.5,
            "native_instances": instances,
            "grasp_adapters": [
                "Two 40 x 45 x 20 mm blue side tabs, 8 g each; no robot attachment"
            ],
            "target_xy": target[:2],
            "target_tolerance_m": 0.035,
            "count": count,
            "ball_radius_m": radius,
            "ball_mass_kg": mass,
            "ball_friction": 0.03,
            "pearl_collision": "outer_sphere",
            "native_inner_profile": BOWL_INNER,
            "top_radius_m": 0.067,
            "bottom_radius_m": 0.032,
            "height_m": 0.054,
            "floor_m": 0.006,
            "wall_segments": 32,
            "lift_m": 0.020,
        },
    )


def marker():
    initial = pose(0.08, 1.14, TABLE_Z + 0.0092, pitch=math.pi / 2)
    body = Article(
        "body",
        (),
        initial,
        initial,
        0.017,
        asset="tuojing:pullable_marker",
        asset_bounds=((-0.0082, -0.0082, 0), (0.0082, 0.0082, 0.128)),
    )
    cap = Article(
        "cap",
        (),
        initial,
        initial,
        0.005,
        asset="tuojing:pullable_marker",
        asset_bounds=((-0.0087, -0.0087, 0.105), (0.0087, 0.0087, 0.151)),
    )
    return BenchSpec(
        replace(
            design("L2-01", "双臂拔马克笔帽", [body, cap]),
            version="l2-01-native-marker-v1",
            provenance="Pinned L20 marker body/cap with four passive radial spring shoes",
        ),
        arms=("left", "right"),
        mechanism="native_radial_spring_marker_fit",
        limitations="Native free body/cap and four passive radial springs. Uncalibrated detent/friction surrogate; prior Sharpa fixed-body test is not R5 bimanual validation. No rigid attachment, material damage or active cap force assistance.",
        facts="The black capped marker lies horizontally on the table, about 151 mm long. Barrel diameter 16.2 mm; cap outside diameter 17.4 mm and length 46 mm. The left hand grips the lower/middle barrel away from the seam; after a verified lift the right hand grips the cap near its closed end, leaving room between the two hands/cameras. Pull the cap axially away from the body; do not unscrew it. Resistance comes from passive radial spring shoes and friction. After full separation, place both parts anywhere safely on the table and release. No target patch, precise final yaw or minimum hand retreat is required. Slight axial rolling does not invalidate a supported placement.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.5,
            "body_tip_z": 0.128,
            "cap_mouth_z": 0.105,
            "cap_end_z": 0.151,
            "body_mass_kg": 0.017,
            "cap_mass_kg": 0.005,
            "native_instances": [
                {
                    "name": "marker",
                    "asset": "pullable_marker",
                    "pose": initial,
                    "tracked": {"body": "Body", "cap": "Cap"},
                }
            ],
        },
    )


def books():
    # The book's 140 mm width cannot fit the R5. Grip the exposed 20 mm
    # thickness, outside both fixed neighbours; preserve full native size/mass.
    x, y = -0.20, 1.08
    # Fixed upper book is a fixture, not a load. A 5 mm gap above the settled
    # middle book avoids turning small controller deflection into a wedge test.
    lower, middle_z, upper = TABLE_Z + 0.150, TABLE_Z + 0.1705, TABLE_Z + 0.195
    initial = pose(x, y + 0.09, middle_z)
    goal = pose(x, y + 0.235, middle_z)
    a = Article(
        "middle_book",
        (),
        initial,
        goal,
        0.3789,
        asset="tuojing:closed_book",
        asset_bounds=((-0.07, -0.11, -0.01), (0.07, 0.11, 0.01)),
    )
    supports = tuple(
        box(
            f"support_{i}", (0.16, 0.025, 0.140), (x, y + offset, TABLE_Z + 0.070), GREY
        )
        for i, offset in enumerate((-0.085, 0.085))
    )
    instances = [
        {
            "name": "lower_book",
            "asset": "closed_book",
            "pose": pose(x, y, lower),
            "static": [""],
            "tracked": {},
        },
        {
            "name": "upper_book",
            "asset": "closed_book",
            "pose": pose(x, y, upper),
            "static": [""],
            "tracked": {},
        },
        {
            "name": "middle_book",
            "asset": "closed_book",
            "pose": initial,
            "tracked": {"middle_book": ""},
        },
    ]
    return BenchSpec(
        replace(
            design("L2-03", "固定上下层的单臂抽书", [a], supports),
            version="l2-03-native-book-clearance-v2",
            release=False,
            provenance="Three native 220 x 140 x 20 mm closed books; middle mass 378.9 g",
        ),
        mechanism="native_book_fixed_neighbours",
        limitations="Closed rigid book, measured-photo mass 378.9 g; 20 mm thickness and friction estimated. Upper/lower books fixed, 5 mm clearance above the settled middle book. No page or cover flex; no support-hand requirement.",
        facts="Three full-size books are stacked horizontally. Upper and lower books are fixed; only the middle book moves. The middle book protrudes 90 mm toward the robots (world +Y). Each book is 220 mm long, 140 mm wide and 20 mm thick. The R5 cannot span the 140 mm width: pinch the exposed front section across its 20 mm THICKNESS, with one pad above and one below, entirely outside the fixed neighbours. Approach from the exposed end, preserve height and pull along +Y until the entire trailing edge clears the stack. About 140 mm of extraction is needed. Holding the extracted book is sufficient; placement or release is not required. The right arm observes only.",
        parameters={
            "table_z": TABLE_Z,
            "stack_front_world": (x, y + 0.11, 0.0),
            "clearance_m": 0.003,
            "minimum_travel_m": 0.13,
            "outward_axis_world": (0.0, 1.0, 0.0),
            "hold_s": 0.5,
            "native_instances": instances,
            "grip_thickness_m": 0.020,
            "exposed_length_m": 0.090,
            "upper_clearance_m": 0.005,
        },
    )


def orange():
    initial, goal = (
        pose(-0.10, 1.17, TABLE_Z + 0.002),
        pose(0.08, 1.10, TABLE_Z + 0.002),
    )
    a = Article(
        "orange",
        (),
        initial,
        goal,
        0.00885472,
        asset="tuojing:orange_segment",
        asset_bounds=((-0.025, -0.02, 0.0), (0.025, 0.01, 0.018)),
    )
    target = replace(
        floor_target(goal[:2], (0.12, 0.12)),
        position=(goal[0], goal[1], TABLE_Z + 0.0002),
    )
    return BenchSpec(
        replace(
            design("L2-06", "软橘子瓣轻夹搬运", [a], [target]),
            version="l2-06-native-fem-orange-v1",
            provenance="Pinned L20 native volume FEM orange; replaces the former eight-cell jelly proxy",
        ),
        mechanism="native_fem_orange_transfer",
        limitations="R5 soft-grip qualification pending: tested grasps slipped or inverted FEM elements; full transfer not certified. Native GPU FEM, estimated mass/material. Deformation limit is an engineering proxy, not a calibrated fracture law. Rupture/juice demo is not run: it rebuilds the scene and cannot be used as a robot controller. Grip/release/support use measured geometry, not invented soft contact forces.",
        facts="Move the exposed orange segment gently onto the green patch and release it. It is about 50 mm long, 26 mm across and 17 mm thick, mass about 9 g, with native elastic deformation. Pinch the fuller middle section across its roughly 26 mm width, avoiding its pointed ends and the tabletop. Close gradually; the R5 minimum 12 mm gap can severely compress this soft object, so do not begin with a fully closed command. Choose the actual opening from images and observed deformation. No rigid container or grasp attachment exists. This task checks a declared 35% deformation reference, not real tissue fracture or juice volume. The right arm may observe only.",
        parameters={
            "table_z": TABLE_Z,
            "hold_s": 0.5,
            "target_xy": goal[:2],
            "target_half": 0.06,
            "soft_asset": "orange_segment",
            "strain_limit": 0.35,
            "deformation_persistence_s": 0.25,
            "lift_m": 0.015,
            "support_height_tolerance_m": 0.0025,
            "finger_clearance_m": 0.004,
        },
    )


def drawer():
    # Native -Y rail is rotated toward the robots (+world Y). The upper drawer
    # provides more table clearance; the lower one is explicitly fixed.
    cabinet = pose(-0.1533, 1.13, TABLE_Z, yaw=math.pi)
    initial = pose(-0.07, 1.13, TABLE_Z + 0.053, yaw=math.pi)
    article = Article(
        "drawer",
        (),
        initial,
        pose(-0.07, 1.21, TABLE_Z + 0.053, yaw=math.pi),
        mass=0.088,
        asset="tuojing:desk_organizer",
        asset_bounds=((-0.0545, -0.124, 0), (0.0545, 0.0695, 0.040)),
    )
    adapter = (
        box("stem", (0.024, 0.042, 0.012), (0, -0.080, 0.026), BLUE),
        box("grip", (0.040, 0.024, 0.020), (0, -0.112, 0.026), BLUE),
    )
    return BenchSpec(
        replace(
            design("L2-07", "单臂拉开固定化妆抽屉", [article]),
            version="l2-07-native-organizer-v1",
            provenance="Pinned L20 desk organizer; native passive 90 mm rail, explicit R5 grip adapter",
        ),
        mechanism="native_prismatic_drawer",
        limitations="Native 90 mm rail and materials; fixed cabinet/lower drawer. Added blue 40 mm R5 grasp tab because the native 8 mm lip cannot be pinched by the configured 12 mm minimum opening. Uncalibrated forces; no damage model.",
        facts="Open only the UPPER drawer of the fixed organizer. The lower drawer and cabinet are fixed. The blue external grip tab is an explicit R5 adapter: pinch across its 40 mm width, keeping fingers clear of the roof and front panel. The drawer opens toward the robots along world +Y. Pull 75-85 mm, never beyond its native 90 mm rail limit; no push latch needs releasing. Release and leave it open. The right arm may observe but not touch. No vertical lift, cabinet rotation, item retrieval or closing is required.",
        parameters={
            "table_z": TABLE_Z,
            "outward_axis_world": (0.0, 1.0, 0.0),
            "required_open_m": 0.075,
            "max_travel_m": 0.09,
            "max_speed_m_s": 0.25,
            "hold_s": 0.5,
            "grasp_adapters": [
                "Blue 40 x 24 x 20 mm grasp tab rigidly belongs to the upper drawer; no robot attachment"
            ],
            "native_instances": [
                {
                    "name": "organizer",
                    "asset": "desk_organizer",
                    "pose": cabinet,
                    "static": ["Body", "Drawer0"],
                    "disabled_joints": ["Joints/Drawer0"],
                    "tracked": {"drawer": "Drawer1"},
                    "adapters": {"Drawer1": adapter},
                    "mass_additions": {"Drawer1": 0.008},
                }
            ],
        },
    )
