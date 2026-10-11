"""L20 task layouts and goals in the delivered laboratory's metre/world frame."""

import math
from dataclasses import replace

from ..l1.specs import Article, box, pose
from .geometry import design
from .spec import BenchSpec

TABLE_Z = 0.78


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
    x, y = 0.19565, 1.30
    # Fixed upper book is a fixture, not a load. A 5 mm gap above the settled
    # middle book avoids turning small controller deflection into a wedge test.
    lower, middle_z, upper = TABLE_Z + 0.080, TABLE_Z + 0.1005, TABLE_Z + 0.125
    initial = pose(x + 0.04, y, middle_z)
    goal = pose(x + 0.15, y, middle_z)
    a = Article(
        "middle_book",
        (),
        initial,
        goal,
        0.3789,
        asset="tuojing:closed_book",
        asset_bounds=((-0.07, -0.11, -0.01), (0.07, 0.11, 0.01)),
    )
    # Photo/video-inspired support, not a calibrated reconstruction. Keep its
    # side flush with the fixed books so the exposed middle edge stays reachable.
    supports = (
        box(
            "blue_support_box",
            (0.14, 0.22, 0.070),
            (x, y, TABLE_Z + 0.035),
            (0.02, 0.36, 0.58),
        ),
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
            design("L2-03", "固定上下层的单臂侧向抽书", [a], supports),
            version="l2-03-centred-side-extraction-v5",
            release=False,
            provenance="Three native 220 x 140 x 20 mm closed books; middle mass 378.9 g",
        ),
        arms=("right",),
        mechanism="native_book_fixed_neighbours",
        limitations="Closed rigid book, measured-photo mass 378.9 g; 20 mm thickness and friction estimated. Upper/lower books fixed, 5 mm clearance above the settled middle book. No page or cover flex; no support-hand requirement.",
        facts="Three full-size books are stacked horizontally at the centre between both arms. Upper and lower books are fixed; only the middle book moves. A blue support box holds the stack; the middle book protrudes 40 mm from its right side in the overview image (world +X). Each book is 220 mm long, 140 mm wide and 20 mm thick. The R5 cannot span the 140 mm width: pinch the exposed right-side section across its 20 mm THICKNESS, with one pad above and one below, entirely outside the fixed neighbours. Approach from the +X side toward the exposed edge, preserve height and pull outward along +X until the entire trailing edge clears the stack. About 110 mm of extraction is needed. After a grasped extraction, support by either the hand or the tabletop is sufficient. No timed hold or continued grasp is required; do not regrasp a fully extracted table-supported book. The RIGHT arm extracts the book from the protruding side; LEFT observes only.",
        parameters={
            "table_z": TABLE_Z,
            "stack_front_world": (x + 0.07, y, 0.0),
            "clearance_m": 0.003,
            "minimum_travel_m": 0.10,
            "outward_axis_world": (1.0, 0.0, 0.0),
            "hold_s": 0.0,
            "native_instances": instances,
            "grip_thickness_m": 0.020,
            "exposed_length_m": 0.040,
            "support_height_m": 0.070,
            "upper_clearance_m": 0.005,
        },
    )


def orange():
    # Keep the setup between both arms; move it 50 mm toward the overview
    # camera (world -Y), with the orange and pedestal moving together.
    pedestal = pose(0.19565, 1.25, TABLE_Z)
    support_z = TABLE_Z + 0.13
    # Rotate in the tabletop plane: the segment's long axis runs front/back
    # in the shared overview, while its original broad underside stays down.
    initial = pose(pedestal[0], pedestal[1], support_z + 0.002, yaw=math.pi / 2)
    goal = pose(initial[0], initial[1], initial[2] + 0.05, yaw=math.pi / 2)
    a = Article(
        "orange",
        (),
        initial,
        goal,
        0.00885472,
        asset="tuojing:orange_segment",
        asset_bounds=((-0.025, -0.02, 0.0), (0.025, 0.01, 0.018)),
    )
    return BenchSpec(
        replace(
            design("L2-06", "从圆柱承台夹起橘子瓣 5 厘米", [a]),
            version="l2-06-centred-forward-lengthwise-lift-v9",
            release=False,
            provenance="Pinned native FEM orange and 2026-10-09 cylindrical pedestal",
        ),
        mechanism="native_fem_orange_lift",
        limitations="Native GPU FEM mesh, dimensions and mass; engineering material overrides (100 kPa, dynamic friction 2.0), not tissue calibration. Native 130 mm pedestal is fixed as a support. Goal measures a 50 mm centroid rise from the settled initial sample while near both left fingers and clear of the support. No placement, release, dwell, speed or deformation limit. No attachments, fracture or juice. Finger proximity is geometry, not measured contact force.",
        facts="Lift the soft orange segment at least 50 mm above its initial resting position on the fixed round pedestal centred between both arms, using LEFT. Keep holding it and stop once raised; no transfer or release is requested. Pedestal: 130 mm high, approximately 68 mm flat top. Orange: 50 x 26 x 17 mm, about 9 g; it lies flat with its long axis along world Y (front/back in the overview). Pinch the fuller middle across its width, with both pads overlapping the sides and clear of the pedestal. Elastic compression and firm closure are allowed; the full 12-70 mm opening range is available. If it slips, support it and close further or correct the grasp. Do not lift an empty hand. RIGHT may move to observe, but not touch the orange or pedestal. There is no green destination patch.",
        parameters={
            "table_z": TABLE_Z,
            "scoring_protocol": "soft_pedestal_lift_v6",
            "soft_asset": "orange_segment",
            "lift_m": 0.05,
            "support_z": support_z,
            "pedestal_asset": "cylindrical_pedestal",
            "pedestal_pose": pedestal,
            "pedestal_height_m": 0.13,
            "pedestal_radius_m": 0.04,
            "finger_proximity_m": 0.003,
        },
    )


def drawer():
    # Native -Y rail is rotated toward the robots (+world Y). The upper drawer
    # provides more table clearance; the lower one is explicitly fixed.
    drawer_x = 0.46131  # About 45 mm image-right of the RIGHT R5 base centre.
    cabinet = pose(drawer_x - 0.0833, 1.13, TABLE_Z, yaw=math.pi)
    initial_open_m = 0.030
    closed = pose(drawer_x, 1.13, TABLE_Z + 0.053, yaw=math.pi)
    initial = pose(drawer_x, closed[1] + initial_open_m, closed[2], yaw=math.pi)
    article = Article(
        "drawer",
        (),
        initial,
        pose(drawer_x, 1.21, TABLE_Z + 0.053, yaw=math.pi),
        mass=0.080,
        asset="tuojing:desk_organizer",
        asset_bounds=((-0.0545, -0.0735, 0), (0.0545, 0.0695, 0.040)),
    )
    return BenchSpec(
        replace(
            design("L2-07", "单臂拉开固定化妆抽屉", [article]),
            version="l2-07-overview-right-arm-v5",
            provenance="Pinned L20 desk organizer; original handle and passive 90 mm rail",
        ),
        arms=("right",),
        mechanism="native_prismatic_drawer",
        limitations="Original native handle, mass, materials and 90 mm rail; fixed cabinet/lower drawer. No added grasp adapter. Native lip is 102 x 6 x 8 mm; reliable grasp with the configured 12-70 mm R5 opening has not been qualified. Uncalibrated forces; no damage model.",
        facts="The organizer stands in front of the RIGHT robot arm as seen in the shared overview image. Use RIGHT to open only its UPPER drawer. The lower drawer and cabinet are fixed. Use the original shallow front lip; there is no added grip tab. The lip is about 102 mm wide, 6 mm deep and 8 mm high. Select contact from observed geometry and the actual gripper range; neither full-width pinching nor closing below the 12 mm minimum is available. Verify physical capture and drawer following rather than assuming a commanded close has grasped the lip. The upper drawer starts 30 mm open to expose the original lip. It opens toward the robots along world +Y. Reach 75-85 mm TOTAL opening measured from fully closed, which means pulling another 45-55 mm from the initial layout, never beyond its native 90 mm rail limit; no push latch needs releasing. Release and leave it open. The LEFT arm may observe but not touch. No vertical lift, cabinet rotation, item retrieval or closing is required.",
        parameters={
            "table_z": TABLE_Z,
            "manipulation_arm": "right",
            "outward_axis_world": (0.0, 1.0, 0.0),
            "closed_pose": closed,
            "initial_open_m": initial_open_m,
            "required_open_m": 0.075,
            "minimum_grasped_pull_m": 0.005,
            "scoring_protocol": "drawer_absolute_opening_v4",
            "max_travel_m": 0.09,
            "max_speed_m_s": 0.25,
            "hold_s": 0.5,
            "native_instances": [
                {
                    "name": "organizer",
                    "asset": "desk_organizer",
                    "pose": cabinet,
                    "static": ["Body", "Drawer0"],
                    "disabled_joints": ["Joints/Drawer0"],
                    "initial_body_offsets": {"Drawer1": (0.0, -initial_open_m, 0.0)},
                    "tracked": {"drawer": "Drawer1"},
                }
            ],
        },
    )
