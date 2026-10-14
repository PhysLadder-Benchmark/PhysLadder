"""Ordered manipulation gates for the opt-in 2026-10-13 L2 revision.

Distances and dwell tolerances are engineering detection thresholds, not real
material calibration. Legacy task scoring is deliberately left untouched.
"""
import numpy as np
from ...robots.transforms import pose_transform
from .evaluation import held, released, quiet, upright, sustained

PROTOCOL = 'ordered_full_extraction_20261013_v1'


def marker(e, rows, legacy):
    _, details, failure = legacy
    p = e.spec.parameters
    body, cap = rows['body'], rows['cap']
    capped = details['tip_in_cap_cavity']
    clear = bool(min(np.min(r['corners'], axis=0)[2] for r in (body, cap)) > p['support_z'] + .008)
    body_held, cap_held = held(body, p['body_arm']), held(cap, p['cap_arm'])
    if capped and not body_held and not cap_held and body['fixture_force_n'] + cap['fixture_force_n'] > .05:
        e.flags.add('strict_capped_on_support')
    stable_pickup = capped and clear and body_held and not cap_held and quiet(body) and quiet(cap)
    if 'strict_capped_on_support' in e.flags and sustained(e, 'strict_capped_hold', stable_pickup, .25):
        e.flags.add('strict_capped_lift_stable')
    # A historical lift cannot qualify a later pickup after setting down or
    # losing the barrel grip. Keep the pre-pull hold continuous.
    if ('strict_active_separation' not in e.flags and
            (not body_held or not clear)):
        e.flags.discard('strict_capped_lift_stable')
        e.flags.discard('strict_pull_armed')
    # Arm the pull only while both hands actually hold the still-capped assembly.
    both = body_held and cap_held and clear
    if 'strict_capped_lift_stable' in e.flags and capped and both:
        e.flags.add('strict_pull_armed')
        e.contact_motion['strict_marker_gap'] = details['current_axial_gap_m']
    gap = details['current_axial_gap_m']
    pulled = ('strict_pull_armed' in e.flags and both and gap >= .002
              and gap - e.contact_motion.get('strict_marker_gap', gap) >= .002
              and not details['recapped'])
    if pulled:
        e.flags.add('strict_active_separation')
    # Loss of either grip before separation invalidates that pull sequence.
    if 'strict_active_separation' not in e.flags and 'strict_pull_armed' in e.flags and not both:
        e.flags.discard('strict_pull_armed')
        e.flags.discard('strict_capped_lift_stable')
    complete = sustained(e, 'strict_final_hold', pulled and quiet(body) and quiet(cap), .25)
    if gap >= .002 and 'strict_active_separation' not in e.flags:
        failure = 'Cap separated without verified capped pickup, stable hold and two-hand active pull'
    details.update(process_protocol=PROTOCOL, current_both_parts_held=both,
                   ordered_process_required=True, stable_hold_s=.25,
                   stages={key:key in e.flags for key in (
                       'strict_capped_on_support','strict_capped_lift_stable',
                       'strict_pull_armed','strict_active_separation')})
    details['stages']['separated_stable_held'] = complete
    return complete, details, failure


def books(e, rows):
    p = e.spec.parameters
    middle = rows['middle_book']
    axis = np.asarray(p['outward_axis_world'], dtype=float)
    axis /= np.linalg.norm(axis)
    neighbours = ('upper_book','lower_book')
    # A 2 mm clearance beyond BOTH neighbouring books replaces the 20 mm travel
    # shortcut. Projection uses measured transformed corners, including rotation.
    middle_min = float(np.min(np.asarray(middle['corners']) @ axis))
    neighbour_max = max(float(np.max(np.asarray(rows[n]['corners']) @ axis)) for n in neighbours)
    clearance = middle_min - neighbour_max
    fully_out = clearance >= .002 and all(middle['pair_forces'].get(n,0.) < .05 for n in neighbours)
    supported = {n for n,r in rows.items() if r['fixture_force_n'] > .05}
    for _ in rows:
        supported.update(n for n,r in rows.items() if any(r['pair_forces'].get(o,0.) > .05 for o in tuple(supported)))
    stable_neighbours = all(n in supported and upright(rows[n], .22) and quiet(rows[n])
                            and min(c[2] for c in rows[n]['corners']) >= p['support_z']-.01
                            for n in neighbours)
    # Allow brief natural settling when the middle support is removed. A sustained
    # table drop is a failed process even if the book stack is restored later.
    dropped = any(rows[n]['bench_force_n'] > .05 and n not in supported for n in neighbours)
    failure = 'Neighbouring book fell off the support during extraction' if sustained(e,'strict_neighbour_drop',dropped,.25) else ''
    if fully_out and held(middle,p['manipulation_arm']) and stable_neighbours:
        e.flags.add('strict_middle_fully_extracted_held')
    placed = bool(released(middle) and quiet(middle) and (middle['bench_force_n'] + middle['fixture_force_n'] > .05))
    final = ('strict_middle_fully_extracted_held' in e.flags and fully_out and stable_neighbours
             and placed and all(released(rows[n]) for n in neighbours))
    complete = sustained(e,'strict_books_final',final,.5)
    travel=float((np.asarray(middle['pose'][:3])-e.initial['middle_book'][:3]) @ axis)
    return complete, dict(process_protocol=PROTOCOL, complete_extraction_required=True,
        outward_travel_m=travel, clearance_m=clearance, required_clearance_m=.002,
        adjacent_books_supported=stable_neighbours, release_required=True,
        target_placement='table or box, fully outside neighbours', final_stability_s=.5,
        stages={'fully_extracted_while_held':'strict_middle_fully_extracted_held' in e.flags,
                'neighbours_stable':stable_neighbours,'released_supported':placed,
                'complete':complete}), failure
