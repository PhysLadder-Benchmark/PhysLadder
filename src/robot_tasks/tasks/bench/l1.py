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
    d = replace(design("L1-01", "白板笔入固定纸杯笔筒", [a], (), friction=0.52),
                fixture_asset="tuojing:ripple_paper_cup", fixture_pose=pose(*holder_xy, table_z),
                version="l1-01-tuojing-marker-cup-v5-finger-contact",
                provenance="Tuojing L20 2026-10-05 locked props; native size, flat marker barrel collider and fixed low-friction cup")
    return BenchSpec(
        d,
        facts="The capped black whiteboard marker is 0.151 m tall; barrel diameter is 0.0162 m and maximum visible width about 0.018 m. The fixed red-brown ripple paper cup serves as the pen holder: height 0.1075 m, inside mouth diameter about 0.0786 m, floor 0.004 m above its bottom. Both are rigid. Keep the cap on. Get the marker into the cup and release it; either end may enter and leaning is allowed. There is no prescribed angle, 40 mm insertion depth or release height. A drop into the cup is allowed if it settles inside and is supported by the cup, without finger or tabletop support, for one simulation second. The initial tabletop placement is valid, but dropping the marker back onto the table or knocking it down onto its side ends the episode as a failure. There is no automatic reset/retry. No absolute prop coordinates are provided.",
        limitations="Photo-estimated native dimensions/mass; rigid fixed paper cup retains delivered static/dynamic contact friction. Flat marker barrel collider replaces a rounded-bottom capsule for upright reset; 0.2 mm contact margin and configured velocity solver iterations. No material deformation or hardware calibration.",
        parameters={
            "table_z": table_z,
            "holder_xy": holder_xy,
            "rim": table_z + 0.1075,
            "floor": table_z + 0.004,
            "radius": 0.025,
            "mouth_radius": 0.0393,
            "marker_visual_radius": 0.00898,  # Visual label/ridges only; never a scoring envelope.
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
                version="l1-03-tuojing-charger-strip-v2-native-score",
                provenance="Tuojing locked USB charger and power strip; native middle two-slot socket, fixed zero-friction fixture")
    return BenchSpec(
        d,
        facts="The white USB charger starts lying on its side. Grip its plastic housing (49 x 27 x 53 mm), not the two metal blades. Insert both blades downward into the MIDDLE blue two-slot socket of the fixed, unpowered strip. Blade spacing is 13.7 mm; each blade is 1.5 x 6.3 mm. Each target slot is 2.6 x 8.2 mm. Native side clearance is only 0.55 mm across blade thickness, so this delivered pair needs finer alignment than the old wide-clearance dummy. There is no latch or powered circuit. Seat the housing at the socket face, release and inspect support.",
        limitations="Photo-estimated rigid charger and fixed strip; native slots retained, 0.55 mm minimum side clearance. Zero fixture friction; no electrical contacts, spring retention or flexible cable. More precise than the former enlarged L1 proxy.",
        parameters={"table_z": table_z, "socket_xy": socket_xy,
                    "slot_centres": ((-0.00685, 0), (0.00685, 0)),
                    "opening": (0.0026, 0.0082), "tongue": (0.0015, 0.0063),
                    "floor": table_z + 0.01315, "rim": table_z + 0.03015,
                    "depth": 0.015, "seat_gap_m": 0.001, "contact_tolerance_m": 0.00015},
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


def hat():
    from ...robots.transforms import pose_transform, transform_pose
    from .native_geometry import hook_geometry
    import numpy as np

    # Present Hook0 toward the left arm (+Y), with a shorter approach from its
    # base. Keep native asset dimensions and the selected support peg intact.
    rack_yaw = 2 * math.pi / 3
    rack_pose = pose(-0.04, 1.09, 0.78, yaw=rack_yaw)
    source_xy = (-0.23, 1.33)
    start, end, radius = hook_geometry(rack_pose)
    direction = (end-start) / np.linalg.norm(end-start)
    up = np.array((0.,0.,1.))
    band_y = up - direction * direction[2]
    band_y /= np.linalg.norm(band_y)
    rotation = np.column_stack((np.cross(band_y,direction),band_y,direction))
    goal = np.eye(4); goal[:3,:3] = rotation
    # The upper rear inner band bears on the shaft, with its round tip beyond
    # the opening. This reference pose is evaluator/planner data, never a policy target.
    goal[:3,3] = start + .70*(end-start) - rotation @ np.array((0.,.098-radius,.035))
    # Turn the brim away from the rack/source-side legs into the open +Y side.
    article = Article('hat', (), pose(*source_xy,.806,yaw=math.pi), transform_pose(goal), .065,
                      asset='tuojing:yellow_visor',
                      asset_bounds=((-0.115833,-0.195,0.0025),(0.115833,0.100,0.087)))
    # A 45-mm passive cradle exposes the underside of the brim for a grasp
    # spanning brim-to-band. The 2-mm band alone cannot be pinched by this gripper.
    rack_transform = pose_transform(rack_pose)
    supports = tuple(replace(
        box('hat_source_pad_'+str(i),(.025,.035,.045),
            tuple((np.array((source_xy[0]+sx*.050,source_xy[1]+sy*.083,.8025))-rack_transform[:3,3])
                  @ rack_transform[:3,:3]), GREY),
        rotation=(0.,0.,-rack_yaw))
        for i,(sx,sy) in enumerate(((-1,-1),(1,-1),(-1,1),(1,1))))
    return BenchSpec(
        replace(design('L1-07','黄色空顶帽挂上木质挂架',[article],supports),
                fixture_asset='tuojing:wooden_rack', fixture_pose=rack_pose,
                version='l1-07-tuojing-native-visor-v4-left-facing-layout',
                provenance='Locked native yellow visor and wooden rack; no scaling, cloth approximation remains rigid'),
        facts="The yellow open-top visor is ONE rigid native prop, about 232 x 295 x 85 mm and 65 g. Its oval head opening is about 176 x 196 mm. The gripper has a nominal 0-70 mm opening. Four 45-mm passive source pads expose the brim underside: pinch the band upper edge from above with vertical distal fingers (one inside, one outside), or pinch an exposed brim edge. Keep the fingers clear of supports and the peg insertion region. Use gentle closure and verify a short lift. Lift the whole visor. Hang its HEAD OPENING on the LOWEST sloping 20-mm peg of the fixed 82-cm wooden rack, letting the rear inner band bear on that peg. Keep the wide brim and both fingers clear of the trunk and other pegs. The round tip must pass beyond the loop, not merely support a rim-balanced hat. Release and inspect actual support on the selected peg. Passive sway is allowed; no stability waiting period.",
        limitations="Photo-estimated native stiff visor represented as one rigid body with 208 separate convex band/brim pieces; no cloth deformation. Native 82-cm rack fixed to bench; 45-mm passive pickup pads only, no hidden attachment or enlarged aperture.",
        parameters={'table_z':.78,'hold_s':0.,'hook_start':tuple(start),'hook_end':tuple(end),
                    'hook_radius':radius,'band_plane_z':.035,
                    'target_hook_suffix':'/Colliders/Hook0',
                    'min_hook_support_n':.065*9.81*.5},
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
                version="l1-08-tuojing-identical-cups-v3-finger-contact",
                provenance="Two copies of the locked native hollow ripple paper cup; no resizing"),
        facts="Two identical empty red-brown ripple cups start upright. Move the cup initially nearer the LEFT robot into the farther cup. Both are movable. Each is 107.5 mm tall, outer mouth about 85 mm wide, lower-body diameter about 56-67 mm within the bottom 40 mm. Gripper opening is nominally 0-70 mm: the full mouth is too wide. A shallow pinch across one rigid wall near the rim is possible from above with vertical fingers, one distal pad inside and one outside. Alternatively grip opposite outer sides of the narrower lower body, with both finger tips clear of the table. Lift and verify support, carry upright, and centre its bottom over the other cup's mouth. A recommended release point is with the cup bottom 5-8 mm INSIDE the mouth and before any finger reaches the rim. Open separately and inspect settling. Opening does not prove detachment; choose a small clear withdrawal probe and inspect whether the cup follows before a full retreat. RGB-D release_check is advisory; the model chooses recovery. Gravity may finish settling the entered cup onto the taper. A 20-35 mm rim separation is typical, not a hard scoring band. Completion uses geometric cavity fit and stable support, without a 3 mm centre cutoff or fixed small-angle gate. No yaw target is required. Temporary tilt or a missed placement can be recovered in the same scene without reset.",
        limitations="Native photo-estimated rigid paper cups, identical taper and size; no paper crushing. Rigid native cup walls; nominal 0-70 mm fingers permit a rim-wall pinch or a lower-body grasp.",
        parameters={"table_z": 0.78, "cup_height": 0.1075, "entry_depth": 0.008,
                    "cup_outer_bottom_radius": 0.028, "cup_outer_top_radius": 0.042,
                    "cup_inner_bottom_radius": 0.025, "cup_inner_top_radius": 0.0393,
                    "cup_floor": 0.004, "cup_contact_tolerance": 0.0015,
                    "min_pair_support_n": 0.014 * 9.81 * 0.5,
                    "min_bench_support_n": 0.028 * 9.81 * 0.5},
    )


def flower():
    # Physics, scoring and carried-object planning share the native bent stem
    # and bud colliders. Do not widen the stem at its tabletop pickup contact.
    table_z, holder_xy = 0.78, (0.105, 1.115)
    rose = Article(
        # Native collider/COM resting pose: both the tip and head start near
        # support, with 0.01 mm clearance, rather than dropping a horizontal rose.
        # Long axis remains along +X. Free dynamics still settle this reset pose.
        "rose", (), (-0.28, 1.22, table_z + 0.0009854593730216,
                     0.2951003515573617, -0.6265583937983512,
                     0.27773423465629116, -0.6657357257737007),
        pose(*holder_xy, table_z + 0.0041), 0.008,
        asset="tuojing:artificial_rose",
        asset_bounds=((-0.0178, -0.0178, -0.000014), (0.0179, 0.0179, 0.3018)),
    )
    d = replace(
        design("L1-09", "刚体玫瑰插花", [rose], ()),
        fixture_asset="tuojing:ripple_paper_cup", fixture_pose=pose(*holder_xy, table_z),
        version="l1-09-tuojing-rigid-rose-v4-settled-reset",
        provenance="Native locked Tuojing rigid artificial rose and fixed ripple paper cup; no scaling or bending joints",
    )
    return BenchSpec(
        d,
        mechanism="rigid_rose_in_fixed_cup",
        facts="The red artificial rose is ONE rigid object about 302 mm long and 8 g, with a thin green stem about 2.1 mm thick and a bud about 34-36 mm wide. It starts lying on the table. The thin stem is difficult to capture reliably; prefer opposite sides of the wider bud or calyx without blocking the free stem tip. The source asset's stem grasp marker is not suitable for this gripper. The entire rose moves rigidly and does not bend. Place the free stem end into the fixed red-brown ripple paper cup (107.5 mm tall, inside mouth about 78.6 mm). Keep the bud above the rim, place the free stem end genuinely inside the cavity, lower gently and release for cup support. No fixed tilt angle or 25 mm insertion depth is required. Natural leaning against the inner wall is allowed. No soft-stem, petal-deformation or fracture behaviour is evaluated.",
        limitations="Photo-estimated native rigid artificial rose and rigid fixed paper cup. Stem, calyx and petals move together; no bending or damage model. Carried-object planning samples the same four bent stem capsules and bud colliders used by native physics and scoring.",
        parameters={"table_z": table_z, "vase_xy": holder_xy,
                    "floor": table_z + 0.004, "rim": table_z + 0.1075,
                    "radius": 0.025, "mouth_radius": 0.0393,
                    "stem_radius": 0.00105,
                    "min_support_n": 0.008 * 9.81 * 0.5},
    )


BUILDERS = {
    "L1-01": marker,
    "L1-03": plug,
    "L1-05": key,
    "L1-07": hat,
    "L1-08": cups,
    "L1-09": flower,
}
