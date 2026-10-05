"""L1 props in the delivered laboratory, with explicit retained proxy exceptions."""

import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, box, pose
from .geometry import cup_parts, cylinder, design, translated
from .spec import BenchSpec


def marker():
    # Native delivery coordinates are bottom-centred, metres, Z-up. The cup is
    # the fixed pen receptacle; no hidden joint is used to keep the marker up.
    table_z, holder_xy = 0.78, (0.095, 1.145)
    a = Article(
        "marker", (), pose(-0.08, 1.16, table_z + 0.001), pose(*holder_xy, table_z + 0.004), 0.022,
        asset="tuojing:whiteboard_marker",
        asset_bounds=((-0.00898, -0.00898, 0.0), (0.00898, 0.00898, 0.1512)),
    )
    d = replace(design("L1-01", "白板笔入固定纸杯笔筒", [a], (), friction=0),
                fixture_asset="tuojing:ripple_paper_cup", fixture_pose=pose(*holder_xy, table_z),
                version="l1-01-tuojing-marker-cup-v2-single-episode",
                provenance="Tuojing L20 2026-10-05 locked props; native size, flat marker barrel collider and fixed low-friction cup")
    return BenchSpec(
        d,
        facts="The capped black whiteboard marker is 0.151 m tall; barrel diameter is 0.0162 m and maximum visible width about 0.018 m. The fixed red-brown ripple paper cup serves as the pen holder: height 0.1075 m, inside mouth diameter about 0.0786 m, floor 0.004 m above its bottom. Both are rigid. Keep the cap on. Get the marker into the cup and release it; either end may enter and leaning is allowed. There is no prescribed angle, 40 mm insertion depth or release height. A drop into the cup is allowed if it settles inside and is supported by the cup, without finger or tabletop support, for one simulation second. The initial tabletop placement is valid, but dropping the marker back onto the table or knocking it down onto its side ends the episode as a failure. There is no automatic reset/retry. No absolute prop coordinates are provided.",
        limitations="Photo-estimated native dimensions/mass; rigid paper cup fixed in place with zero insertion friction. Flat marker barrel collider replaces a rounded-bottom capsule for upright reset; no material deformation or hardware calibration.",
        parameters={
            "table_z": table_z,
            "holder_xy": holder_xy,
            "rim": table_z + 0.1075,
            "floor": table_z + 0.004,
            "radius": 0.025,
            "mouth_radius": 0.0393,
            "marker_radius": 0.00898,
            "marker_height": 0.1512,
            "min_support_n": 0.022 * 9.81 * 0.5,
        },
    )


def plug():
    table_z, socket_xy = 0.78, (0.07, 1.10)
    # These payload samples follow the three native convex colliders. A single
    # enclosing box would fill the gap between the two blades in the planner.
    parts = (
        box("housing", (0.049, 0.027, 0.053), (0, 0, 0.0435), BLUE),
        box("blade_left", (0.0015, 0.0063, 0.018), (-0.00685, 0, 0.009), GREY),
        box("blade_right", (0.0015, 0.0063, 0.018), (0.00685, 0, 0.009), GREY),
    )
    a = Article(
        "plug", parts, pose(-0.12, 1.23, table_z + 0.0255, pitch=math.pi / 2),
        pose(*socket_xy, table_z + 0.01315), 0.055,
        asset="tuojing:usb_charger",
        asset_bounds=((-0.0245, -0.01355, 0), (0.0245, 0.0135, 0.07075)),
    )
    d = replace(design("L1-03", "USB 充电器插入固定插线板", [a], (), friction=0),
                fixture_asset="tuojing:power_strip", fixture_pose=pose(*socket_xy, table_z),
                version="l1-03-tuojing-charger-strip-v1",
                provenance="Tuojing locked USB charger and power strip; native middle two-slot socket, fixed zero-friction fixture")
    return BenchSpec(
        d,
        facts="The white USB charger starts lying on its side. Grip its plastic housing (49 x 27 x 53 mm), not the two metal blades. Insert both blades downward into the MIDDLE blue two-slot socket of the fixed, unpowered strip. Blade spacing is 13.7 mm; each blade is 1.5 x 6.3 mm. Each target slot is 2.6 x 8.2 mm. Native side clearance is only 0.55 mm across blade thickness, so this delivered pair needs finer alignment than the old wide-clearance dummy. There is no latch or powered circuit. Seat the housing at the socket face, release and inspect support.",
        limitations="Photo-estimated rigid charger and fixed strip; native slots retained, 0.55 mm minimum side clearance. Zero fixture friction; no electrical contacts, spring retention or flexible cable. More precise than the former enlarged L1 proxy.",
        parameters={"table_z": table_z, "socket_xy": socket_xy,
                    "slot_centres": ((-0.00685, 0), (0.00685, 0)),
                    "opening": (0.0026, 0.0082), "tongue": (0.0015, 0.0063),
                    "floor": table_z + 0.01315, "rim": table_z + 0.03015,
                    "depth": 0.015},
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
        (-0.10, 1.20, 0.78),
        "source_",
    )
    target = translated(
        rectangular_well((0, 0), (0.018, 0.028), 0.032, 0.004), (0.07, 1.14, 0.78), "lock_"
    )
    a = Article("key", parts, pose(-0.10, 1.20, 0.785), pose(0.07, 1.14, 0.784), 0.028)
    return BenchSpec(
        replace(design("L1-05", "钥匙插锁（柄部外露）", [a], source + target, friction=0), version="l1-05-tuojing-layout-proxy-v1"),
        facts="The yellow key bow protrudes above its grey source cradle. Lift it out, align the flat blade with the fixed target keyway and insert without turning. The bow may remain held at completion.",
        limitations="Enlarged flat dummy key/keyway; no lock tumblers or unlocking mechanism.",
        parameters={
            "table_z": 0.78,
            "socket_xy": (0.07, 1.14),
            "opening": (0.018, 0.028),
            "tongue": (0.010, 0.018),
            "floor": 0.784,
            "rim": 0.816,
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
    # Native 82 cm rack: use the lowest sloping peg, rotated toward the arms.
    from ...robots.transforms import pose_transform
    import numpy as np

    # A diagonal peg remains approachable from the left arm and is visible
    # beside the trunk in the fixed overview, instead of hiding behind it.
    rack_pose = pose(0.09, 1.00, 0.78, yaw=5 * math.pi / 12)
    t = pose_transform(rack_pose)
    centre = np.array((0.05932374016062806, -0.03425, 0.3825))
    direction = np.array((0.721558830, -0.416591606, 0.552997706))
    half_length = 0.115624481620995 / 2
    start, end = [t[:3, :3] @ (centre + sign * direction * half_length) + t[:3, 3]
                  for sign in (-1, 1)]
    contact = start + 0.70 * (end - start)
    # Cup-local Y faces along the peg; its handle remains in local XZ.
    cup_yaw = -math.pi / 4
    centre_offset = np.array((0.042 * math.cos(cup_yaw), 0.042 * math.sin(cup_yaw), 0.04))
    a = Article("mug", parts, pose(-0.12, 1.27, 0.781),
                pose(*(contact - centre_offset), yaw=cup_yaw), 0.060)
    return BenchSpec(
        replace(design("L1-07", "塑料杯挂上木质挂架", [a], ()),
                fixture_asset="tuojing:wooden_rack", fixture_pose=rack_pose,
                version="l1-07-tuojing-rack-proxy-mug-v1",
                provenance="Native Tuojing wooden rack, lowest peg; retained project hollow handled cup"),
        facts="The red rigid cup has a rectangular loop handle (opening 30 x 32 mm). Its body is about 54 mm wide. Hang through the HANDLE, not the cup mouth, on the LOWEST sloping peg of the fixed wooden rack. That peg is 20 mm thick and points upward and toward the robots; the higher pegs are not targets. Lift the handle clear of the rounded tip, pass it over the tip and lower gently onto the peg. Leave the handle hole unobstructed by the grasp. Success requires release and actual rack support above the table; passive sway is allowed, with no stability waiting period.",
        limitations="Native photo-estimated 82 cm wooden rack fixed to bench; no delivered handled cup, so the original rigid hollow cup is retained. No material damage model.",
        parameters={"table_z": 0.78, "hold_s": 0.0,
                    "hook_start": tuple(start), "hook_end": tuple(end), "hook_radius": 0.01,
                    "handle_centre": (0.042, 0, 0.037), "handle_half": (0.015, 0.016)},
    )


def cups():
    # Both cups retain the same native dimensions, hollow walls and 14 g mass.
    bounds = ((-0.0426, -0.0426, 0), (0.0426, 0.0426, 0.107594535))
    # Hollow payload samples preserve the opening instead of filling it with an AABB.
    parts = cup_parts(bottom_radius=0.025, top_radius=0.0393, height=0.1075,
                      wall=0.003, segments=48)
    lower = Article("lower_cup", parts, pose(0.07, 1.13, 0.781), pose(0.07, 1.13, 0.78),
                    0.014, asset="tuojing:ripple_paper_cup", asset_bounds=bounds)
    upper = Article("upper_cup", parts, pose(-0.10, 1.22, 0.781), pose(0.07, 1.13, 0.805),
                    0.014, asset="tuojing:ripple_paper_cup", asset_bounds=bounds)
    return BenchSpec(
        replace(design("L1-08", "两个波纹纸杯套叠", [lower, upper]),
                version="l1-08-tuojing-identical-cups-v1",
                provenance="Two copies of the locked native hollow ripple paper cup; no resizing"),
        facts="Two identical empty red-brown ripple cups start upright. Move the cup initially nearer the LEFT robot into the farther cup. Both are movable. Each is 107.5 mm tall, outer mouth about 85 mm wide, lower-body diameter about 56-67 mm within the bottom 40 mm. Gripper opening is 12-70 mm: do not grasp across the wide mouth or assume it can pinch a 3 mm wall. Grip opposite outer sides of the narrower lower body, with both finger tips clear of the table. Lift and verify support, carry upright, and centre its bottom over the other cup's mouth. A recommended release point is with the cup bottom 5-8 mm INSIDE the mouth and before any finger reaches the rim. Open separately and withdraw; gravity may finish settling the entered cup onto the taper. A 20-35 mm rim separation is typical, not a hard scoring band. Completion uses geometric cavity fit and stable support, without a 3 mm centre cutoff or fixed small-angle gate. No yaw target is required. Temporary tilt or a missed placement can be recovered in the same scene without reset.",
        limitations="Native photo-estimated rigid paper cups, identical taper and size; no paper crushing. Lower-body grasp followed by shallow entry and passive settling; thin-wall pinch is incompatible with the current 12 mm minimum opening.",
        parameters={"table_z": 0.78, "cup_height": 0.1075, "entry_depth": 0.008,
                    "cup_outer_bottom_radius": 0.028, "cup_outer_top_radius": 0.042,
                    "cup_inner_bottom_radius": 0.025, "cup_inner_top_radius": 0.0393,
                    "cup_floor": 0.004, "cup_contact_tolerance": 0.0015,
                    "min_pair_support_n": 0.014 * 9.81 * 0.5,
                    "min_bench_support_n": 0.028 * 9.81 * 0.5},
    )


def flower():
    # Native rose is one rigid body; the narrow stem and wider bud remain
    # distinct in the conservative planner proxy as well as in native physics.
    table_z, holder_xy = 0.78, (0.105, 1.115)
    parts = (
        cylinder("stem_proxy", 0.003, 0.265, (0, 0, 0.1325), GREEN),
        cylinder("bud_proxy", 0.018, 0.055, (0, 0, 0.2745), RED),
    )
    rose = Article(
        "rose", parts, pose(-0.28, 1.22, table_z + 0.020, pitch=math.pi / 2),
        pose(*holder_xy, table_z + 0.0041), 0.008,
        asset="tuojing:artificial_rose",
        asset_bounds=((-0.0178, -0.0178, -0.000014), (0.0179, 0.0179, 0.3018)),
    )
    d = replace(
        design("L1-09", "刚体玫瑰插花", [rose], ()),
        fixture_asset="tuojing:ripple_paper_cup", fixture_pose=pose(*holder_xy, table_z),
        version="l1-09-tuojing-rigid-rose-v1",
        provenance="Native locked Tuojing rigid artificial rose and fixed ripple paper cup; no scaling or bending joints",
    )
    return BenchSpec(
        d,
        mechanism="rigid_rose_in_fixed_cup",
        facts="The red artificial rose is ONE rigid object about 302 mm long and 8 g, with a thin green stem about 2.1 mm thick and a bud about 34-36 mm wide. It starts lying on the table. The 12 mm minimum gripper opening cannot pinch the thin stem; grasp opposite sides of the wider bud or calyx instead, without blocking the free stem tip. The source asset's stem grasp marker is not suitable for this gripper. The entire rose moves rigidly and does not bend. Place the free stem end into the fixed red-brown ripple paper cup (107.5 mm tall, inside mouth about 78.6 mm). Keep the bud above the rim, place the free stem end genuinely inside the cavity, lower gently and release for cup support. No fixed tilt angle or 25 mm insertion depth is required. Natural leaning against the inner wall is allowed. No soft-stem, petal-deformation or fracture behaviour is evaluated.",
        limitations="Photo-estimated native rigid artificial rose and rigid fixed paper cup. Stem, calyx and petals move together; no bending or damage model. Narrow stem/wider bud proxy is used only for planner clearance; physical colliders remain native.",
        parameters={"table_z": table_z, "vase_xy": holder_xy,
                    "floor": table_z + 0.004, "rim": table_z + 0.1075,
                    "radius": 0.025, "mouth_radius": 0.0393,
                    "stem_radius": 0.00105, "stem_length": 0.237,
                    "head_start": 0.247, "head_radius": 0.018,
                    "min_support_n": 0.008 * 9.81 * 0.5},
    )


BUILDERS = {
    "L1-01": marker,
    "L1-03": plug,
    "L1-05": key,
    "L1-07": mug,
    "L1-08": cups,
    "L1-09": flower,
}
