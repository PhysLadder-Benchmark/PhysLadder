"""Feishu real-trial prompt protocol. Scoring evidence never selects robot actions.

The source contains instruction summaries, not a hardware system prompt. Time,
home and handoff tolerances below are explicit simulation operationalizations.
Legacy geometry builders remain usable for isolated geometric regression tests.
"""

from dataclasses import replace
import math

import numpy as np

from ...robots.transforms import pose_transform, transform_pose
from ..l1.specs import pose
from .evaluation import held, released, sustained, upright
from .scoring_l1 import stable_outcome

PROTOCOL = "feishu_real_l1_scene_v3_real_arm_names"
TASKS = {"L1-01", "L1-03", "L1-07", "L1-08", "L1-09"}


def configure(spec, robot):
    task = spec.design.task_id
    if task not in TASKS:
        raise ValueError(f"No verified real prompt for {task}")
    p = {**spec.parameters, "scoring_protocol": PROTOCOL, "real_prompt_protocol": True,
         "hold_s": 1.0, "home_tolerance_rad": .05, "home_velocity_rad_s": .05,
         "home_positions": {m.name: tuple(m.home_positions_rad) for m in robot.mounts},
         "observation_arms": ("right",) if task != "L1-08" else (),
         "return_home_arms": ("left", "right") if task in {"L1-01", "L1-08", "L1-09"} else (),
         "stability_position_tolerance_m": .005, "stability_tilt_tolerance_rad": math.radians(5),
         "protocol_limitations": "Document scene plus verbatim trial instruction. Correct-arm contact history, actual geometry, independent release/support and requested return are checked; no prescribed lift height or TCP-box gap. Stability verification allows 5 mm / 5 degrees for 1 s; return tolerance 0.05 rad / 0.05 rad/s. These are simulation tolerances, not real-trial precision. Hat sway is allowed without a dwell; its unspecified previous grasp side is not scored."}
    design = replace(spec.design, version=spec.design.version + "-real-scene-v2", release=True)
    limitations = spec.limitations
    if task == "L1-07":
        p["hold_s"] = 0.0
    if task == "L1-09":
        from .l1 import plug

        articles = {a.name: a for a in design.articles}
        cup = articles['source_cup']
        # User clarified overview image coordinates: flower/source on the RIGHT,
        # destination on the LEFT. Do not interpret robot arm names as image sides.
        source_xy, target_xy = cup.initial[:2], design.fixture_pose[:2]
        source_pose = pose(*source_xy, p['table_z'] + .001)
        target_pose = pose(*target_xy, p['table_z'] + .001)
        charger = next(a for a in plug().design.articles if a.name == 'plug')
        # Measured charger body: local z=17..67 mm; two metal pins at z=0.
        # Pins face UP; rotate the wide (40 mm) side along cup Y so the
        # overview-left channel can contain the flower without crossing the wall.
        # Initial geometry validated against the measured 70/50/80-mm cup.
        # Small positive contact clearances; free bodies, no weld or pose servo.
        source_frame = pose_transform(source_pose)
        charger_pose = transform_pose(source_frame @ pose_transform(pose(.003, 0., .076, roll=math.pi, yaw=math.pi/2)))
        flower_pose = transform_pose(source_frame @ pose_transform(pose(-.0095, 0., .0042, pitch=math.radians(-9))) @ pose_transform(pose(0., 0., 0., yaw=math.radians(110))))
        # User correction 2026-10-14: flower transfer has an EMPTY destination cup.
        design = replace(design, name='奶白郁金香从画面右源杯移入左目标杯',
                         version=spec.design.version + '-real-scene-v7-empty-flower-target',
                         fixture_asset='', articles=(
            replace(articles['rose'], initial=flower_pose, goal=pose(*target_xy,p['table_z']+.004)),
            replace(cup, initial=source_pose, goal=source_pose),
            replace(cup, name='target_cup', initial=target_pose, goal=target_pose),
            replace(charger, name='source_charger', initial=charger_pose, goal=charger_pose)))
        limitations = ('Native rigid tulip and two movable paper cups. Overview-right source contains the native charger; '
                        'two metal pins up and USB port down; flower near vertical on the charger\'s overview-left side. No inferred guide or attachment. '
                        'Overview-left target is empty initially; no marbles or pearls in the flower-transfer task.')
        p.update(target_article='target_cup', source_max_tilt_rad=math.radians(12), source_charger_pins_min_up=math.cos(math.radians(20)),
                 source_ignore_axial_spin=False,
                 source_support_vertical=True,
                 source_support_articles=('source_cup','source_charger'),
                 target_fillers=(),
                 source_rest_tolerances=dict(position_tolerance_m=.003, tilt_tolerance_rad=math.radians(3),
                                             linear_speed_m_s=.01, angular_speed_rad_s=.15),
                 source_guide_status='native charger inside source cup; no inferred stem guide',
                 vase_xy=target_xy, target_filler_status='empty target cup; user correction 2026-10-14',
                 alignment_gaps=('camera intrinsics and unmeasured initial positions remain estimates',
                                 'initial pose checked for visual and collision-mesh intersections; dynamics and support stability not yet verified'))
    if task == "L1-03":
        # The three native matching parallel-blade pairs, in strip coordinates.
        # Outer sockets also have angled holes; those do not match this charger.
        candidates = ((-.01, -.064, math.pi / 2), (0., 0., 0.), (-.01, .052, math.pi / 2))
        fixture = pose_transform(design.fixture_pose)
        centres = [(fixture @ np.array((x, y, .03015, 1)))[:3] for x, y, _ in candidates]
        index = 1  # Latest C5 explicitly identifies the middle blue two-slot socket.
        x, y, yaw = candidates[index]
        offset = np.array((math.cos(yaw), math.sin(yaw))) * .00685
        p.update(slot_centres=tuple(tuple(np.array((x, y)) + sign * offset) for sign in (-1, 1)),
                 opening=(.0082, .0026) if index != 1 else (.0026, .0082),
                 socket_xy=tuple(centres[index][:2]), target_socket_index=index,
                 target_socket_selection="explicit middle blue two-slot socket in Feishu C5")
        goal = np.eye(4)
        goal[:3, :3] = ((math.cos(yaw), -math.sin(yaw), 0), (math.sin(yaw), math.cos(yaw), 0), (0, 0, 1))
        goal[:3, 3] = (x, y, .01315)
        design = replace(design, articles=tuple(replace(a, goal=transform_pose(fixture @ goal))
                         if a.name == "plug" else a for a in design.articles))
    if task == "L1-08":
        by_name = {a.name: a for a in design.articles}
        # Same two physical starting positions, with identities matching the
        # real sequence: robot-left (overview RIGHT) moves the receiver first;
        # robot-right (overview LEFT) then nests the other cup into it.
        first, second = by_name["lower_cup"].initial, by_name["upper_cup"].initial
        # C11 supplies the missing central handoff intent. Use a broad region
        # relative to the initial spacing, never an exact 6-cm target circle.
        centre = (np.asarray(first[:2]) + second[:2]) / 2
        p.update(scoring_protocol="feishu_real_l1_08_scene_v4_real_arm_names", sequential_arms=True,
                 handoff_start_xy=first[:2], handoff_end_xy=second[:2],
                 handoff_progress_range=(.25, .75), handoff_lateral_fraction=.5,
                 stability_position_tolerance_m=.005,
                 stability_tilt_tolerance_rad=math.radians(5),
                 protocol_limitations="L1-08 central handoff from C11: first cup moves into the broad middle half of initial cup spacing, lateral allowance half the spacing; no precise point, minimum lift height, nesting depth or yaw target. Early preparation is recoverable. Independent support/release verified for 1 s, allowing 5 mm / 5 degrees drift; home tolerance 0.05 rad / 0.05 rad/s. These are explicit simulation verification choices, not measured task precision.")
        design = replace(design, version=spec.design.version + "-real-scene-v3", articles=tuple(
            replace(a, initial=first if a.name == "lower_cup" else second,
                    goal=(*centre, p["table_z"] + (0 if a.name == "lower_cup" else .025), 1., 0., 0., 0.))
            for a in design.articles))
    design = replace(design, version=design.version + "-real-arm-names")
    return replace(spec, design=design, arms=("left", "right") if task == "L1-08" else ("left",),
                   parameters=p, facts="", limitations=limitations + " " + p["protocol_limitations"])


def home_evidence(p, robot):
    result = {}
    for arm, target in p["home_positions"].items():
        state = robot.get(arm)
        q = np.asarray(state.joint_positions_rad if state else (), dtype=float)
        v = np.asarray(state.joint_velocities_rad_s if state else (), dtype=float)
        valid = q.shape == v.shape == (len(target),) and np.isfinite(q).all() and np.isfinite(v).all()
        error = float(np.max(np.abs(q - target))) if valid else None
        speed = float(np.max(np.abs(v))) if valid else None
        result[arm] = {"valid": bool(valid), "max_joint_error_rad": error,
                       "max_joint_speed_rad_s": speed,
                       "at_home": bool(valid and error <= p["home_tolerance_rad"] and speed <= p["home_velocity_rad_s"])}
    return result


def requirements(e, rows, robot, geometry_done, details, failure):
    """Add measured release/stability, correct-arm process and return-home gates."""
    p = e.spec.parameters
    task = e.spec.design.task_id
    home = home_evidence(p, robot or {})
    valid_robot = all(r["valid"] for r in home.values())
    robot = robot or {}
    stages = {}
    violations = []
    # Record handling by the requested arm, without prescribing lift height.
    targets = (("lower_cup", "left"), ("upper_cup", "right")) if task == "L1-08" else (
        ({"L1-01": "marker", "L1-03": "plug", "L1-07": "hat", "L1-09": "rose"}[task], "left"),)
    for name, arm in targets:
        eligible = task != "L1-08" or name == "lower_cup" or "first_cup_placed_and_left_home" in e.flags
        if sustained(e, "real_handled_" + name, eligible and held(rows[name], arm), .15):
            e.flags.add("real_handled_" + name)

    # Keep a coarse TCP separation diagnostic for debugging only. It is not a
    # completion gate: real release is measured through contacts and support.
    withdrawn = {}
    for name, arm in targets:
        state = robot.get(arm)
        tcp = state.tcp_pose_world_wxyz if state else None
        article = next(a for a in e.spec.design.articles if a.name == name)
        clear = False
        if tcp is not None and len(tcp) == 7 and np.isfinite(tcp).all():
            local = (np.linalg.inv(pose_transform(rows[name]["pose"])) @ np.array((*tcp[:3], 1)))[:3]
            lo, hi = np.asarray(article.asset_bounds)
            distance = np.linalg.norm(np.maximum(np.maximum(lo-local, local-hi), 0.))
            clear = distance >= .015
        withdrawn[name] = bool(clear)

    if task == "L1-08":
        base, top = rows["lower_cup"], rows["upper_cup"]
        stage_one = "first_cup_placed_and_left_home" in e.flags
        if sustained(e, "wrong_cup_arm", held(base, "right") or held(top, "left"), .15):
            violations.append("Cup manipulated by the wrong arm for the real-trial sequence")
        moving = {a: r["valid"] and r["max_joint_speed_rad_s"] > .05 for a, r in home.items()}
        if sustained(e, "simultaneous_arms", all(moving.values()), .15):
            violations.append("Both arms moved simultaneously in a sequential task")
        if not stage_one and sustained(e, "right_started_early", moving["right"] or held(top, "right"), .15):
            violations.append("Right arm started before first-cup placement and left return")
        if stage_one and sustained(e, "left_restarted", moving["left"] or held(base, "left"), .15):
            violations.append("Left arm resumed manipulation during the right-arm stage")
        start, end = np.asarray(p['handoff_start_xy']), np.asarray(p['handoff_end_xy'])
        delta = end-start; spacing = np.linalg.norm(delta)
        offset = np.asarray(base['pose'][:2])-start
        progress = float(np.dot(offset,delta) / spacing**2)
        lateral = float(np.linalg.norm(offset-progress*delta))
        in_handoff = p['handoff_progress_range'][0] <= progress <= p['handoff_progress_range'][1] and lateral <= spacing*p['handoff_lateral_fraction']
        details = {**details, 'handoff_progress': progress, 'first_cup_in_handoff_region': bool(in_handoff)}
        placed = ("real_handled_lower_cup" in e.flags and in_handoff and released(base) and upright(base)
                  and base["bench_force_n"] > .014 * 9.81 * .5)
        placement_stable = stable_outcome(e, "first_cup", [base], placed)
        if sustained(e, "first_placement_stable", placement_stable, p["hold_s"]) and home["left"]["at_home"]:
            e.flags.add("first_cup_placed_and_left_home")
        stages = {"first_cup_handled_by_left": "real_handled_lower_cup" in e.flags,
                  "first_cup_placed_and_left_home": "first_cup_placed_and_left_home" in e.flags,
                  "second_cup_handled_by_right": "real_handled_upper_cup" in e.flags}
        process_ok = all(stages.values())
        # The cup geometry rule already checks release/support and positional stability.
        # Full release and home return already establish separation. No extra
        # TCP-to-bounding-box distance is required by the cup instruction.
        outcome = bool(geometry_done and process_ok)
        # Report recoverable ordering mistakes, but do not end an otherwise
        # recoverable episode on joint velocities during a preparatory move.
        details = {**details, "process_warnings": violations}
        violations = []
    else:
        name, _ = targets[0]
        row = rows[name]
        support = details.get("fixture_supported", details.get("target_hook_supported", False))
        outcome = bool(geometry_done and released(row) and support)
        stable_rows = [row] + ([rows[p['target_article']]] if p.get('target_article') else [])
        if task != 'L1-07':
            outcome = stable_outcome(e, "real_outcome", stable_rows, outcome)
        process_ok = "real_handled_" + name in e.flags
        stages["handled_by_left"] = process_ok
    if violations:
        e.flags.add("real_process_violated")
    stable = sustained(e, "real_outcome_confirmed", outcome and process_ok, p["hold_s"])
    homes_ok = all(home[a]["at_home"] for a in p["return_home_arms"])
    stages.update(independently_stable=bool(stable), required_arms_home=bool(homes_ok))
    complete = bool(stable and homes_ok and valid_robot and "real_process_violated" not in e.flags)
    details = {**details, "release_required": True, "stability": "continuous independent support and position/tilt verification",
               "stability_wait_s": p["hold_s"], "independent_stable_duration_s": (
                   e.last_time-e.timers["real_outcome_confirmed"] if "real_outcome_confirmed" in e.timers else 0.),
               "home": home, "withdrawn": withdrawn, "stages": stages,
               "constraint_violations": [*details.get("constraint_violations", []), *violations],
               "protocol": p["scoring_protocol"]}
    return complete, details, failure or ("; ".join(violations))
