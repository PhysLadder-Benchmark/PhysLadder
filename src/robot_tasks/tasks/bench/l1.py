"""L1 props in the delivered laboratory, with explicit retained proxy exceptions."""

import math
from dataclasses import replace

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, box, pose
from .geometry import cup_parts, cylinder, design, translated
from .spec import BenchSpec


def marker():
    from .native_geometry import cup_cavity

    # Native delivery coordinates are bottom-centred, metres, Z-up. The cup is
    # the fixed pen receptacle; no hidden joint is used to keep the marker up.
    # Looking from the front of the table (-Y): mug on image left, marker on
    # image right, at the same depth. Photo-proportional, not a measured calibration.
    table_z, holder_xy = 0.78, (0.04, 1.22)
    _, cup_floor, cup_rim = cup_cavity('black_ceramic_mug_20261013_reference_v3')
    a = Article(
        "marker", (), pose(0.34, 1.22, table_z + 0.001), pose(*holder_xy, table_z + cup_floor), 0.022,
        asset="tuojing:capped_marker_20261011",
        asset_bounds=((-0.0087, -0.0087, 0.0), (0.0087, 0.0087, 0.151)),
    )
    d = replace(design("L1-01", "白板笔入固定黑色陶瓷杯", [a], (), friction=0.52),
                fixture_asset="tuojing:black_ceramic_mug_20261013_reference_v3",
                fixture_pose=pose(*holder_xy, table_z, yaw=-math.pi / 2),
                version="l1-01-tuojing-marker-ceramic-mug-v12-reference-85mm-2717g",
                provenance="Feishu 2026-10-11 marker and ceramic mug; photo-proportional front-view layout, handle toward table front")
    return BenchSpec(
        d,
        facts="The capped black whiteboard marker is 0.151 m tall; barrel diameter is 0.0162 m and maximum visible width about 0.018 m. The fixed black ceramic mug with white interior serves as the pen holder: measured height 0.085 m and mass 0.2717 kg; reference-shape-derived inside mouth diameter about 0.08028 m, floor 0.006611 m above its bottom. Its middle is wider than its mouth and base, with an open handle; the target is the main cup cavity, not the handle hole. Both are rigid. Keep the cap on. L1 success is genuine entry of either marker end into the main cavity, without significant wall or floor penetration. Holding the marker, moving and leaning are allowed. Releasing above the opening and letting the marker fall into the cavity is also valid; release alone is not success, actual entry must occur. Release, cup support, bottom seating and a stable waiting period are not required. There is no prescribed angle or 40 mm insertion depth. Merely aligning above the mouth or resting across its rim is insufficient. Returning to or toppling on the tabletop permits recovery in the same scene; falling below the table still ends the episode. There is no automatic reset. No absolute prop coordinates are provided.",
        limitations="Cup height and mass measured by user (85 mm, 271.7 g); remaining dimensions, contact parameters and initial layout are photo estimates; fixed ceramic mug matches the latest real-task C3 description and retains delivered contact friction. Body and cap form one rigid closed marker for L1; internal cap shoes/springs disabled. No hand attachment, material deformation or hardware calibration.",
        parameters={
            "scoring_protocol": "l1_entry_only_v1",
            "hold_s": 0.0,
            "table_z": table_z,
            "holder_xy": holder_xy,
            "rim": table_z + cup_rim,
            "floor": table_z + cup_floor,
            "radius": 0.029 * 85 / 90,
            "mouth_radius": 0.0425 * 85 / 90,
            "marker_visual_radius": 0.0087,  # Visual label/ridges only; never a scoring envelope.
            "marker_height": 0.151,
            "min_support_n": 0.022 * 9.81 * 0.5,
        },
    )


def plug():
    table_z, socket_xy = 0.78, (0.02, 1.20)
    source_xy = (0.35, 1.20)
    strip_yaw = math.pi / 2
    # These payload samples follow the three native convex colliders. A single
    # enclosing box would fill the gap between the two blades in the planner.
    parts = (
        box("housing", (0.040, 0.020, 0.050), (0, 0, 0.042), BLUE),
        box("blade_left", (0.0015, 0.0063, 0.018), (-0.00685, 0, 0.009), GREY),
        box("blade_right", (0.0015, 0.0063, 0.018), (0.00685, 0, 0.009), GREY),
    )
    a = Article(
        "plug", parts, pose(*source_xy, table_z + 0.0295),
        pose(socket_xy[0] + 0.0135, socket_xy[1], table_z + 0.00515, yaw=strip_yaw), 0.055,
        asset="tuojing:usb_charger_measured_20261014",
        asset_bounds=((-0.020, -0.01005, 0), (0.020, 0.010, 0.06775)),
    )
    # The native 20.8-g holder is free-standing, not attached to the charger or
    # table. The photo variant raises its shoulder seat from 20 to 45 mm;
    # the 22-mm through-hole remains open for the blades. The whole charger is
    # unchanged and settles naturally with about 40 mm of housing exposed.
    base_pose = pose(*source_xy, table_z + 0.001)
    base = Article("plug_storage_base", (), base_pose, base_pose, 0.0208,
                   asset="tuojing:plug_storage_base",
                   asset_bounds=((-0.0507, -0.0507, 0), (0.0507, 0.0507, 0.055)))
    d = replace(design("L1-03", "USB 充电器插入固定插线板", [a, base], (), friction=0),
                fixture_asset="tuojing:power_strip_measured_20261014", fixture_pose=pose(*socket_xy, table_z, yaw=strip_yaw),
                version="l1-03-measured-envelope-spring-20261014",
                release=False,
                provenance="Tuojing locked charger, strip and 2026-10-09 free-standing plug holder; photo-proportional strip-left/charger-right layout, not calibrated")
    return BenchSpec(
        d,
        facts="The white USB charger starts upright, blades downward, naturally supported in a black recessed storage base on the right side of the table. The free-standing base is about 101 x 101 x 55 mm and 20.8 g, with a 22-mm central through-hole. Its 10-mm-deep recess exposes about 40 mm of the housing above the holder; it is passive storage, not the target socket. Grip the exposed upper plastic housing (whole housing 40 x 20 x 50 mm in asset XYZ; user measured length 50, width 40, thickness 20 mm excluding blades), not the base or metal blades. Lift vertically clear of the recess before translation or rotation; the base should remain on the table. The fixed unpowered white strip lies horizontally on the left, with its coiled cable behind it. Insert both blades downward into the MIDDLE blue two-slot socket. Blade spacing is 13.7 mm; each blade is 1.5 x 6.3 mm. Each target slot is 2.6 x 8.2 mm. Native side clearance is only 0.55 mm across blade thickness. There is no latch or powered circuit. L1 success requires both blades genuinely entering their corresponding middle slots without significant wall, floor or housing penetration. Holding, motion and partial insertion are allowed. No 15-mm insertion depth, housing seating, release, support force, grasp history or stable waiting period is required. Alignment above the face, rim contact and only one blade entering are insufficient. Entry must exceed the existing 0.15-mm numerical contact tolerance; this is not a task-depth target.",
        limitations="Measured strip body 165 x 50 mm, total mass including rigid bundled cable 280.4 g, and charger body 50 x 40 x 20 mm. Strip body height 22 mm, charger mass 55 g, blade/slot dimensions and contact parameters remain estimates; no real force calibration. Free-standing 20.8-g storage base uses a photo-estimated shallow pocket: floor raised from 20 to 45 mm, recess 35 to 10 mm, visuals and colliders changed together while retaining outer dimensions and blade through-hole; shared task object friction is 0.8. Fixed strip keeps native slots, 0.55 mm minimum side clearance and the new native housing/contact materials. Native passive spring contacts are retained with their own friction and collision skins; housing fixed kinematically. No powered circuit or flexible cable.",
        parameters={"table_z": table_z, "socket_xy": socket_xy,
                    "scoring_protocol": "l1_plug_entry_only_v1", "hold_s": 0.0,
                    "slot_centres": ((-0.00685, -0.0135), (0.00685, -0.0135)),
                    "opening": (0.0026, 0.0082), "tongue": (0.0015, 0.0063),
                    "floor": table_z + 0.00515, "rim": table_z + 0.02215,
                    "seat_gap_m": 0.001, "contact_tolerance_m": 0.00015},
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
    visor_scale_y = 0.8056400439652093
    visor_scale_z = 0.8
    from ...robots.transforms import transform_pose
    from .native_geometry import hook_geometry
    import numpy as np

    # Front-view photo: rack left, supported visor right; lowest peg faces the
    # right/rear robot-side approach corridor. Rotate the native -30-degree peg
    # heading to +30 degrees toward the right arm. Metre scale is not calibrated.
    rack_yaw = math.pi / 3
    # Keep the rack on the left, but clear its rear leg from the home fingers.
    rack_pose = pose(-0.10, 1.10, 0.78, yaw=rack_yaw)
    source_xy = (0.40, 1.17)
    start, end, radius = hook_geometry(rack_pose, 'wooden_rack_20261014_measured')
    direction = (end-start) / np.linalg.norm(end-start)
    up = np.array((0.,0.,1.))
    band_y = up - direction * direction[2]
    band_y /= np.linalg.norm(band_y)
    rotation = np.column_stack((np.cross(band_y,direction),band_y,direction))
    goal = np.eye(4); goal[:3,:3] = rotation
    # The upper rear inner band bears on the shaft, with its round tip beyond
    # the opening. This reference pose is evaluator/planner data, never a policy target.
    goal[:3,3] = start + .70*(end-start) - rotation @ np.array((0.,.098*visor_scale_y-radius,.035*visor_scale_z))
    # Native brim extends toward -Y, the open front of the table.
    article = Article('hat', (), pose(*source_xy,.857-.0024*visor_scale_z), transform_pose(goal), .065,
                      asset='tuojing:yellow_visor_20261014_measured',
                      asset_bounds=((-0.0949999988079071, -0.15838883817195892, 0.0019200000679120421), (0.0949999988079071, 0.08161116391420364, 0.07063999772071838)))
    box_pose = pose(source_xy[0], source_xy[1]-.035, .781)
    source_box = Article('source_box', (), box_pose, box_pose, .2608,
                         asset='tuojing:green_support_box_20261014_measured',
                         asset_bounds=((-0.1025,-0.0975,0),(0.1025,0.0975,0.075)))
    return BenchSpec(
        replace(design('L1-07','黄色空顶帽挂上木质挂架',[article, source_box],()),
                fixture_asset='tuojing:wooden_rack_20261014_measured', fixture_pose=rack_pose,
                version='l1-07-tuojing-visor-v9-measured-hat-box',
                provenance='User-measured visor footprint 190 x 240 mm and box 205 x 195 x 75 mm; original meshes retained; rigid cloth approximation'),
        facts="The yellow open-top visor is ONE rigid native prop, 190 mm wide x 240 mm long; height about 69 mm (unmeasured) and an estimated 65 g. Its oval head opening is about 142.3 x 157.9 mm (scaled original, not measured). The width and overall front-back length are user measured; visuals and colliders agree. The gripper has a nominal 0-70 mm opening. The visor starts on a free-standing teal support box, 205 x 195 x 75 mm, on the right of the rack; the brim faces the table front. The box is passive storage, not the target. Where the box leaves clearance, pinch the band upper edge from above with vertical distal fingers (one inside, one outside), or pinch an exposed brim edge. Keep the fingers clear of supports and the peg insertion region. Use gentle closure and verify a short lift. Lift the whole visor. Hang its HEAD OPENING on the LOWEST sloping 20-mm peg of the fixed 74-cm wooden rack (each complete hook rod is 90 mm along its axis), letting the rear inner band bear on that peg. The lowest peg points toward the right/rear robot side of the table. Keep the wide brim and both fingers clear of the trunk and other pegs. The round tip must pass beyond the loop, not merely support a rim-balanced hat. Release and inspect actual support on the selected peg. Passive sway is allowed; no stability waiting period.",
        limitations="Original stiff visor adjusted to measured width 190 mm and overall length 240 mm, represented as one rigid body with 208 separate convex band/brim pieces; no cloth deformation. Measured 74-cm rack with 90-mm axial hook rods fixed to bench; measured 75-mm-high free-standing support box, no hidden attachment or enlarged aperture.",
        parameters={'table_z':.78,'hold_s':0.,'hook_start':tuple(start),'hook_end':tuple(end),
                    'hook_radius':radius,'band_plane_z':.035*visor_scale_z,
                    'target_hook_suffix':'/Colliders/Hook0',
                    'min_hook_support_n':.065*9.81*.5},
    )


def cups():
    # Both cups retain the same native dimensions, hollow walls and 14 g mass.
    # Photo-proportional layout, not calibrated: viewed from the table front,
    # the source is on the left and the receiver is 30 cm right, 2 cm forward.
    source_xy, receiver_xy, table_z = (0.04, 1.20), (0.34, 1.18), 0.78
    bounds = ((-.035, -.035, 0), (.035, .035, .080))
    # Hollow payload samples preserve the opening instead of filling it with an AABB.
    parts = cup_parts(bottom_radius=0.021865670347679562, top_radius=0.03163432329893112, height=0.07992970943450928,
                      wall=0.003, segments=48)
    lower = Article("lower_cup", parts, pose(*receiver_xy, table_z + 0.001), pose(*receiver_xy, table_z),
                    0.014, asset="tuojing:ripple_paper_cup_20261013_measured", asset_bounds=bounds)
    upper = Article("upper_cup", parts, pose(*source_xy, table_z + 0.001), pose(*receiver_xy, table_z + 0.025),
                    0.014, asset="tuojing:ripple_paper_cup_20261013_measured", asset_bounds=bounds)
    return BenchSpec(
        replace(design("L1-08", "两个波纹纸杯套叠", [lower, upper]),
                version="l1-08-tuojing-identical-cups-v6-measured-70-50-80",
                provenance="Two native hollow ripple paper cups; photo-proportional left-source/right-receiver layout, receiver slightly toward the table front; measured cup dimensions 70/50/80 mm; original mesh topology retained"),
        facts="Two identical empty red-brown ripple cups start upright. Viewed from the table front toward the robots, move the initially LEFT cup into the initially RIGHT cup, which starts slightly toward the table front. Both are movable. Each has measured height 80 mm, outer mouth diameter 70 mm and base diameter 50 mm. Gripper opening is nominally 0-70 mm: the full mouth leaves no approach clearance. A shallow pinch across one rigid wall near the rim is possible from above with vertical fingers, one distal pad inside and one outside. Alternatively grip opposite outer sides of the narrower lower body, with both finger tips clear of the table. Lift and verify support, carry upright, and centre its bottom over the other cup's mouth. A recommended release point is with the cup bottom 5-8 mm INSIDE the mouth and before any finger reaches the rim. Open separately and inspect settling. Opening does not prove detachment; choose a small clear withdrawal probe and inspect whether the cup follows before a full retreat. If release_check feedback is enabled, it is advisory; the model chooses recovery. Gravity may finish settling the entered cup onto the taper. A 20-35 mm rim separation is typical, not a hard scoring band. Completion uses geometric cavity fit and stable support, without a 3 mm centre cutoff or fixed small-angle gate. No yaw target is required. Temporary tilt or a missed placement can be recovered in the same scene without reset.",
        limitations="Measured 70/50/80 mm rigid paper cups with original estimated 14 g mass, identical taper and size; no paper crushing. Rigid native cup walls; nominal 0-70 mm fingers permit a rim-wall pinch or a lower-body grasp.",
        parameters={"table_z": 0.78, "cup_height": 0.080, "entry_depth": 0.008,
                    "cup_outer_bottom_radius": 0.025, "cup_outer_top_radius": 0.035,
                    "cup_inner_bottom_radius": 0.021865670347679562, "cup_inner_top_radius": 0.03163432329893112,
                    "cup_floor": 0.0029741288162767887, "cup_contact_tolerance": 0.0015,
                    "min_pair_support_n": 0.014 * 9.81 * 0.5,
                    "min_bench_support_n": 0.028 * 9.81 * 0.5},
    )


def flower():
    # Front-view reference: empty destination left, upright flower in a second
    # cup right. The source cup is free-standing, destination fixed as before.
    table_z, holder_xy, source_xy = .78, (.00, 1.05), (.44, 1.05)
    cup_bounds = ((-.035,-.035,0),(.035,.035,.080))
    source_pose = pose(*source_xy, table_z+.001)
    source_cup = Article('source_cup', (), source_pose, source_pose, .014,
                        asset='tuojing:ripple_paper_cup_20261013_measured', asset_bounds=cup_bounds)
    # Explicit rigid surrogate for the pale source-cup insert seen in the
    # photo; exact real material is unconfirmed. Open 6-mm bore, no joint or
    # gripping force. It is a separate 4-g body resting inside the source cup.
    from .geometry import closed_piece
    from ..l1.specs import Part
    guide_parts=[]
    for i in range(32):
        vertices=[]
        for angle in (i*2*math.pi/32,(i+1)*2*math.pi/32):
            for radius,z in ((.003,0),(.0171,0),(.003,.0558),(.0253,.0558)):
                vertices.append((radius*math.cos(angle),radius*math.sin(angle),z))
        vertices,faces=closed_piece(vertices)
        guide_parts.append(Part('sector_'+str(i),(1.,1.,1.),(0,0,0),(0,0,0),
                                'mesh',(.9,.9,.85),vertices=tuple(map(tuple,vertices)),
                                faces=tuple(tuple(map(int,face)) for face in faces)))
    guide_pose=pose(*source_xy,table_z+0.004474128816276788)
    guide=Article('source_stem_guide',tuple(guide_parts),guide_pose,guide_pose,.004)
    initial=pose(*source_xy,table_z+0.004474128816276788)
    flower = Article('rose', (), initial, pose(*holder_xy,table_z+0.0029741288162767887), .012,
                     asset='tuojing:cream_tulip_20261011',
                     asset_bounds=((-0.03143,-0.01767,.001),(.03623,.02450,.300)))
    d = replace(design('L1-09', '奶白郁金香从右杯移入左杯', [flower, source_cup, guide], ()),
                fixture_asset='tuojing:ripple_paper_cup_20261013_measured',
                fixture_pose=pose(*holder_xy,table_z),
                version='l1-09-tuojing-tulip-v6-measured-cups',
                provenance='Feishu 2026-10-11 cream tulip and colour-corrected cups; photo-proportional layout, not calibrated')
    return BenchSpec(
        d, mechanism='rigid_tulip_transfer_between_cups',
        facts="The cream-white tulip starts approximately upright in the RIGHT red-brown source cup. Transfer the FLOWER into the initially empty LEFT destination cup. The source cup and its pale insert are free-standing; the insert has an open bore and only supports the initial stem mechanically. Leave both in place. The destination cup is fixed. The whole flower is one rigid body, about 300 mm tall and 12 g, with a 3.8-mm stem, 46-mm bud and two leaves. Prefer a gentle grasp on the wider flower head; do not pinch the thin stem or expect leaves/petals to bend. Verify a short lift, then raise until the entire stem tip clears the source rim before moving sideways. Keep the stem approximately downward. Place the free stem end genuinely inside the left cup, lower gently, release, and verify that the destination supports it with the flower head above its rim. Both cups have measured height 80 mm, outer mouth 70 mm and base 50 mm; inside mouth is approximately 63.3 mm. Remaining in or returning to the source cup is not completion. Natural leaning in the destination is allowed. No fixed tilt or 25-mm insertion depth is required.",
        limitations="Native 2026-10-11 rigid tulip with stem, leaf and petal colliders; no bending, crushing or damage model. Free-standing source cup with a separate rigid 4-g pale insert (55.8 mm high, open 6-mm bore), inferred from the photo; real insert material is unconfirmed. No flower joint or forced pose. Fixed destination cup; photo-proportional layout without metric camera calibration. Physics/scoring/carried-object planning use the same native flower geometry.",
        parameters={'table_z':table_z,'vase_xy':holder_xy,
                    'floor':table_z+0.0029741288162767887,'rim':table_z+0.07992970943450928,
                    'radius':0.021865670347679562,'mouth_radius':0.03163432329893112,'stem_radius':.0019,
                    'source_support_articles':('source_cup','source_stem_guide'),
                    'source_guide_status':'photo-inferred rigid insert; real material unconfirmed',
                    'min_support_n':.012*9.81*.5},
    )


BUILDERS = {
    "L1-01": marker,
    "L1-03": plug,
    "L1-05": key,
    "L1-07": hat,
    "L1-08": cups,
    "L1-09": flower,
}
