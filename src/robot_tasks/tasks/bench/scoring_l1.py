"""Native collider containment, release and load-bearing criteria for L1."""

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import held, quiet, relative, released
from .native_geometry import (band_opening, cavity_fit, clip_below, cup_cavity,
                              shapes, transformed_poly)


def stable_outcome(e, key, rows, active, *, relative_frame=None):
    """One-second outcome stability ignores yaw/spin but measures position/tilt.

    Contact/release/containment must hold at every sample. Bounded drift from
    the interval's first pose prevents slow translation or precession passing.
    """
    if not hasattr(e,'outcome_anchors'): e.outcome_anchors={}
    if not active:
        e.outcome_anchors.pop(key,None)
        return False
    states=[]
    for row in rows:
        transform=relative(row,relative_frame) if relative_frame is not None else pose_transform(row['pose'])
        axis=pose_transform(row['pose'])[:3,2]
        transverse=np.asarray(row['angular_velocity'])-axis*np.dot(row['angular_velocity'],axis)
        if np.linalg.norm(row['linear_velocity']) >= .025 or np.linalg.norm(transverse) >= .3:
            e.outcome_anchors.pop(key,None)
            return False
        states.append((transform[:3,3],transform[:3,2]))
    previous=e.outcome_anchors.get(key)
    if previous is not None and any(np.linalg.norm(pos-old_pos) > .001 or
                                    np.dot(axis,old_axis) < np.cos(.02)
                                    for (pos,axis),(old_pos,old_axis) in zip(states,previous)):
        e.outcome_anchors.pop(key,None)
        return False
    if previous is None:
        e.outcome_anchors[key]=[(pos.copy(),axis.copy()) for pos,axis in states]
    return True


def marker(e, rows):
    r,p=rows['marker'],e.spec.parameters
    t=pose_transform(r['pose'])
    cavity=t.copy(); cavity[:2,3]-=p['holder_xy']; cavity[2,3]-=p['table_z']
    inside,margin=cavity_fit('whiteboard_marker',cavity,.001)
    vertices=np.concatenate([transformed_poly('whiteboard_marker',i,t)[0]
                             for i in range(len(shapes('whiteboard_marker')))])
    depth=p['rim']-vertices[:,2].min()
    support=r['fixture_force_n'] > p['min_support_n'] and r['bench_force_n'] < .05
    if vertices[:,2].min() > p['table_z']+.005 and r['bench_force_n'] < .05:
        e.flags.add('marker_left_table')
    body_on_table=abs(t[2,2]) < .5 and vertices[:,2].min() <= p['table_z']+.003
    if (not held(r) and r['bench_force_n']>.05 and
            ('marker_left_table' in e.flags or body_on_table)):
        e.flags.add('marker_dropped_on_table')
    failure='Marker dropped onto the tabletop' if 'marker_dropped_on_table' in e.flags else ''
    done=stable_outcome(e,'marker',[r],inside and support and released(r))
    return done, {'inside_holder':inside,'depth_m':float(depth),'fixture_supported':bool(support),
                  'collider_clearance_m':float(margin),'geometry':'native barrel cylinder + cap capsule',
                  'dropped_on_table':bool(failure),'stability':'position and tilt; axial spin ignored'}, failure


def insertion(e, rows):
    if e.spec.design.task_id == "L1-03":
        return plug(e, rows)
    name = "key"
    r = rows[name]
    p = e.spec.parameters
    t = pose_transform(r["pose"])
    axis = t[:3, 2]
    depth = p["rim"] - t[2, 3]
    rim = t[:3, 3] + axis * (depth / max(axis[2], 1e-6))
    # Cross-section projected into the rectangular keyway; 180-degree yaw symmetry is natural.
    # Section in the horizontal socket plane; account for tilt geometrically
    # rather than imposing a separate angle limit.
    section = t[:2, :2] - np.outer(axis[:2], t[2, :2]) / max(axis[2], 1e-6)
    extent = np.abs(section) @ (np.array(p["tongue"]) / 2)
    if "slot_centres" in p:
        # Match both native blades to both native holes, including 180-degree
        # symmetry. A body-centre-only test would accept a pin outside its slot.
        slots = np.array(p["slot_centres"])
        tips = np.array([t[:2, 3] + t[:2, :2] @ c for c in slots])
        at_rim = tips + axis[:2] * depth / max(axis[2], 1e-6)
        centres = []
        for order in (slots, slots[::-1]):
            offset = np.vstack((tips - order - p["socket_xy"], at_rim - order - p["socket_xy"]))
            centres.append(offset)
        cross_section_fits = any(np.all(np.abs(c) + extent <= np.array(p["opening"]) / 2 + 0.00015)
                                 for c in centres)
    else:
        centres = np.vstack((t[:2, 3], rim[:2])) - p["socket_xy"]
        cross_section_fits = np.all(np.abs(centres) + extent <= np.array(p["opening"]) / 2 + 0.0005)
    inside = bool(
        axis[2] > 1e-6
        and depth >= p["depth"]
        and t[2, 3] >= p["floor"] - 0.002
        and cross_section_fits
    )
    release_ok = name == "key" or released(r)
    support = held(r) or r["fixture_force_n"] > 0.05
    return (
        inside and name in e.grasped and release_ok and support and quiet(r),
        {
            "inside_socket": inside,
            "depth_m": float(depth),
            "release_required": name == "plug",
        },
        "",
    )



def plug(e, rows):
    r,p=rows['plug'],e.spec.parameters
    t=pose_transform(r['pose']); tol=p['contact_tolerance_m']
    slots=np.asarray(p['slot_centres']) + p['socket_xy']
    slot_half=np.asarray(p['opening'])/2
    blades=[]
    for i,s in enumerate(shapes('usb_charger')):
        if not s['name'].startswith('Blade'):continue
        vertices,edges,_=transformed_poly('usb_charger',i,t)
        clipped=clip_below(vertices,edges,p['rim'])
        blades.append((vertices,clipped))
    assignments=[]
    for order in (slots,slots[::-1]):
        checks=[]
        for (vertices,clipped),centre in zip(blades,order):
            depth=p['rim']-vertices[:,2].min()
            fit=bool(len(clipped) and np.all(np.abs(clipped[:,:2]-centre)<=slot_half+tol)
                     and vertices[:,2].min()>=p['floor']-tol and depth>=p['depth'])
            checks.append({'inside_slot':fit,'depth_m':float(depth)})
        assignments.append(checks)
    checks=max(assignments,key=lambda a:sum(c['inside_slot'] for c in a))
    housing_index=next(i for i,s in enumerate(shapes('usb_charger')) if s['name']=='Housing')
    housing=transformed_poly('usb_charger',housing_index,t)[0]
    # Inspect the authored housing's broad shoulder, not only a blade tip or
    # a chamfer's lowest corner: a tilted charger may contact without seating.
    local=transformed_poly('usb_charger',housing_index,np.eye(4))[0]
    shoulder=housing[np.isclose(local[:,2],local[:,2].min(),atol=1e-8)]
    gaps=shoulder[:,2]-p['rim']
    seated=bool(t[2,2]>0 and gaps.min()>=-tol and gaps.max()<=p['seat_gap_m'])
    inside=bool(all(c['inside_slot'] for c in checks))
    support=r['fixture_force_n']>.05 and r['bench_force_n']<.05
    done=inside and seated and released(r) and support and quiet(r)
    return done, {'inside_socket':inside,'blades':checks,'housing_seated':seated,
                  'housing_seat_gap_range_m':[float(gaps.min()),float(gaps.max())],
                  'depth_m':min(c['depth_m'] for c in checks),'release_required':True,
                  'grasp_lift_recorded':'plug' in e.grasped,'native_dimensions_retained':True}, ''


def hat(e, rows):
    r,p=rows['hat'],e.spec.parameters
    t=pose_transform(r['pose'])
    a=(np.asarray(p['hook_start'])-t[:3,3]) @ t[:3,:3]
    b=(np.asarray(p['hook_end'])-t[:3,3]) @ t[:3,:3]
    direction=(b-a)/np.linalg.norm(b-a); z=p['band_plane_z']; radius=p['hook_radius']
    through=False
    if abs(direction[2]) > 1e-6:
        f=(z-a[2])/(b[2]-a[2]); hit=a+f*(b-a)
        planes=band_opening(); normals=planes[:,:2]
        extent=radius*np.sqrt(np.sum(normals**2,axis=1)+(normals @ direction[:2]/direction[2])**2)
        # Both spherical caps must be past the opening plane: balancing the
        # rear band on the rounded end is insufficient, even with contact.
        caps_clear=min(a[2],b[2])+radius<z<max(a[2],b[2])-radius
        through=bool(0<f<1 and caps_clear and np.all(normals@hit[:2]+extent<=planes[:,2]+.001))
    support=(r.get('target_hook_vertical_force_n',0)>p['min_hook_support_n']
             and r['bench_force_n']<.05 and np.min(np.asarray(r['corners'])[:,2])>p['table_z']+.03)
    done=through and support and released(r)
    return done, {'hook_through_head_opening':through,'target_hook_supported':bool(support),
                  'target_hook_vertical_force_n':r.get('target_hook_vertical_force_n',0),
                  'release_required':True,'stability_wait_s':0.,'geometry':'native 64-sector band and rounded Hook0'}, ''


def cups(e, rows):
    top,base=rows['upper_cup'],rows['lower_cup']; p=e.spec.parameters
    t=relative(top,base)
    contained,margin=cavity_fit('ripple_paper_cup',t,p['cup_contact_tolerance'])
    floor_index=next(i for i,s in enumerate(shapes('ripple_paper_cup')) if s['name']=='Floor')
    bottom=transformed_poly('ripple_paper_cup',floor_index,t)[0]
    contained=bool(contained and t[2,2]>0 and pose_transform(base['pose'])[2,2]>0
                   and bottom[:,2].max()<cup_cavity()[2]-p['cup_contact_tolerance'])
    supported=(top['pair_forces'].get('lower_cup',0)>p['min_pair_support_n']
               and base['bench_force_n']>p['min_bench_support_n'])
    active=contained and supported and released(top) and released(base)
    # Relative nesting geometry and the base's world pose must both stay put.
    pair_stable=stable_outcome(e,'cup_pair',[top],active,relative_frame=base)
    base_stable=stable_outcome(e,'cup_base',[base],active)
    return pair_stable and base_stable, {'nested':contained,'cup_on_cup_support':bool(supported),
                 'relative_bottom_height_m':float(t[2,3]),'collider_clearance_m':float(margin),
                 'geometry':'native segmented cup walls and floor',
                 'stability':'relative position and tilt; axial spin ignored'}, ''


def flower(e, rows):
    r,p=rows['rose'],e.spec.parameters; t=pose_transform(r['pose'])
    cavity=t.copy(); cavity[:2,3]-=p['vase_xy']; cavity[2,3]-=p['table_z']
    stems=[i for i,s in enumerate(shapes('artificial_rose')) if s['name'].startswith('Stem')]
    # Check all immersed stem segments, including bends. The bud's complete
    # collision envelope must remain above the mouth.
    inside,margin=cavity_fit('artificial_rose',cavity,.001,indices=stems)
    tip=transformed_poly('artificial_rose',stems[0],t)[0]
    inside=bool(inside and tip[:,2].min()<p['rim']-.001 and tip[:,2].max()>p['floor']-.001)
    heads=[i for i,s in enumerate(shapes('artificial_rose')) if not s['name'].startswith('Stem')]
    head_envelopes=[transformed_poly('artificial_rose',i,t) for i in heads]
    head_low=min(vertices[:,2].min()-error for vertices,_,error in head_envelopes)
    head_clear=bool(head_low>p['rim'] and r['bench_force_n']<.01)
    supported=bool(r['fixture_force_n']>p['min_support_n'])
    active=inside and head_clear and supported and released(r)
    done=stable_outcome(e,'rose',[r],active)
    return done, {'stem_end_inside':inside,'depth_m':float(p['rim']-tip[:,2].min()),
                  'flower_clear_of_table':head_clear,'head_above_rim':head_clear,
                  'fixture_supported':supported,'rigid_flower':True,'collider_clearance_m':float(margin),
                  'geometry':'four native bent stem capsules and native bud colliders',
                  'grasp_lift_recorded':'rose' in e.grasped,
                  'stability':'position and tilt; axial spin ignored if containment persists'}, ''


RULES={'L1-01':marker,'L1-03':insertion,'L1-05':insertion,'L1-07':hat,'L1-08':cups,'L1-09':flower}
