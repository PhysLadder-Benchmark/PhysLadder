"""Opt-in Feishu L2 alignment. Static evidence is not dynamics validation."""
import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path

PROFILE = 'feishu_l2_20261013'
TASKS = ('L2-01','L2-03','L2-04','L2-06','L2-07')

def configure_drawer_blocks(stage, parent, config):
    """Opt-in initial contents; source assets stay unchanged and bodies stay free."""
    if config.task_id != 'L2-07' or config.raw['task'].get('real2sim_profile') != PROFILE:
        return
    from pxr import Gf, UsdGeom, UsdShade
    positions = [(-.109,-.0825),(-.075,-.0825),(-.1173,-.044),(-.0833,-.044),(-.0493,-.044)]
    for i in range(1, 6):
        prim = stage.GetPrimAtPath(parent + '/Contents/Cube%02d' % i)
        xf = UsdGeom.Xformable(prim)
        # Two near the opening, three occluded farther inside, as in the video.
        # Base stays on the floor; no artificial levitation or floating cubes.
        xf.MakeMatrixXform().Set(Gf.Matrix4d().SetTranslate(
            Gf.Vec3d(*positions[i-1], .0551)))
        xf.AddScaleOp().Set(Gf.Vec3f(1.5, 1.5, 1.2 if i==2 else 1.6))
        shader = UsdShade.Shader(stage.GetPrimAtPath(str(prim.GetPath()) + '/Looks/Blue/Surface'))
        shader.GetInput('diffuseColor').Set(Gf.Vec3f(.018,.18,.30) if i in (2,3,5) else Gf.Vec3f(.03,.36,.17))
        prim.SetCustomDataByKey('asset_label', 'Blue/green block 30x30x24-32mm; photo-estimated visual revision')
        prim.SetCustomDataByKey('dimensions_status', 'User requested taller visible blocks; photo estimate, not measured')
        prim.SetCustomDataByKey('mass_status', 'Original 2.6g retained; taller revision mass uncalibrated')

def verified_asset(name, config):
    from ...asset_delivery import verified_prop
    from ...paths import PROJECT_ROOT
    if config.raw['task'].get('real2sim_profile') == PROFILE and config.task_id == 'L2-07' and name == 'green_support_box':
        path,evidence=verified_prop(name,task_id='L2-03')
        return path,{**evidence,'task_id':'L2-07','reused_verified_bundle':'L2-03/support_box'}
    mapping = {
        'closed_book': ('books', 'closed_book/closed_book.usda'),
        'orange_segment': ('orange', 'orange_rounded/orange_three_connected_spherical.usda'),
        'desk_organizer': ('drawer', 'drawer_contents/organizer_with_contents.usda'),
        **{'marble_'+v: ('marbles', 'assets/marble_'+v+'/marble_'+v+'.usda') for v in
           ('yellow_green','aqua_blue','emerald_green','clear_mint','red_white')},
    }
    if config.raw['task'].get('real2sim_profile') != PROFILE or name not in mapping:
        return verified_prop(name, task_id=config.task_id)
    assert config.task_id in TASKS
    root=PROJECT_ROOT/'assets/real2sim_l2_20261013'
    package, entry=mapping[name]
    lock=json.loads((root/'source.lock.json').read_text())[package]
    for filename,digest in lock['files'].items():
        if hashlib.sha256((root/package/filename).read_bytes()).hexdigest()!=digest:
            raise ValueError('L2 asset digest mismatch: '+filename)
    return root/package/entry, {'delivery':'2026-10-11','prop':name,'task_id':config.task_id,
        'package':package,'sha256':lock['files'][entry], 'archive_sha256':lock['archive_sha256'],
        'installed_path':str(root/package/entry), 'profile':PROFILE}

def align_spec(spec, task):
    from ..l1.specs import Article, pose
    from .pearl_pouring import cup_contains
    from .native_geometry import cup_cavity
    if task.get('real2sim_profile') != PROFILE or spec.design.task_id not in TASKS:
        raise ValueError('Unsupported L2 alignment profile')
    p=dict(spec.parameters)
    articles=list(spec.design.articles)
    facts=spec.facts
    tid=spec.design.task_id
    if tid=='L2-03':
        articles=[replace(a,initial=pose(*a.initial[:2],.890+i*.020),
            goal=pose(*a.goal[:2],.890+i*.020),
            asset_bounds=((-0.07,-0.11,-.010),(.07,.11,.010))) for i,a in enumerate(articles)]
        p['native_instances']=[dict(it,pose=next(a.initial for a in articles if a.name==it['name']))
            if it['name']!='support_box' else dict(it) for it in p['native_instances']]
        facts=facts.replace('19.6 mm','20 mm')
    elif tid=='L2-04':
        radius=.008; mass=.00536165146212658
        p['cup_asset']='ripple_paper_cup_20261013_measured'
        articles=[replace(a,asset='tuojing:'+p['cup_asset'],asset_bounds=((-.035,-.035,0),(.035,.035,.08))) if i<2 else a for i,a in enumerate(articles)]
        source,receiver=articles[0].initial,articles[1].initial
        floor=cup_cavity(p.get('cup_asset','ripple_paper_cup'))[1]
        points=[]; spacing=.0165
        for layer in range(4):
            z=floor+radius+.0004+layer*spacing
            candidates=[(spacing*(i+j/2),spacing*math.sqrt(3)/2*j,z) for i in range(-3,4) for j in range(-3,4)]
            for point in sorted(candidates,key=lambda q:q[0]**2+q[1]**2):
                if cup_contains(point,radius=radius,tolerance=-.0003,cup_asset=p['cup_asset']):
                    points.append(point)
                    if len(points)==5: break
            if len(points)==5: break
        assert len(points)==5
        articles=articles[:2]; instances=[dict(i,asset=p['cup_asset']) for i in p['native_instances'][:2]]
        for i,(pt,color) in enumerate(zip(points,('yellow_green','aqua_blue','emerald_green','clear_mint','red_white'))):
            name=f'load_{i}'; asset='marble_'+color
            initial=pose(*(source[j]+pt[j] for j in range(3)))
            goal=pose(*(receiver[j]+pt[j] for j in range(3)))
            articles.append(Article(name,(),initial,goal,mass,asset='tuojing:'+asset,
                asset_bounds=((-radius,)*3,(radius,)*3)))
            instances.append(dict(name=name,asset=asset,pose=initial,tracked={name:''}))
        p.update(native_instances=instances,count=5,required_received=5,minimum_loaded_lift_count=5,
                 ball_radius_m=radius,ball_mass_kg=mass,loaded_source_mass_kg=.014+5*mass,
                 initial_packing_top_m=max(q[2] for q in points)+radius,
                 scoring_protocol='five_16mm_marbles_all_received_v1',finish_when_all_received=True)
        p.pop('ball_friction',None);p.pop('pearl_collision',None)
        facts=('The RIGHT source cup contains exactly FIVE free 16 mm glass marbles, one of each delivered colour. '
               'The LEFT receiver starts empty. RIGHT lifts the source and pours into the receiver; LEFT observes only. '
               'Success requires all FIVE marbles settled entirely inside the receiver (5/5). Four is failure; no residue allowance. '
               'Use the native hollow cups, 80 mm high with 70 mm rim and 50 mm base; grasp the lower body from the side. '
               'Keep fingers, wrist and camera clear during tilting. All five received is the success condition; source placement and robot homing are not additional success gates. '
               'Receiver is a fixed fixture in this simulation. Marbles keep native sphere collision and glass material. '
               'Total source mass is approximately 40.81 g; material/contact parameters remain uncalibrated.')
    elif tid=='L2-07':
        p.update(required_open_m=p['initial_open_m']+.005,return_tolerance_m=.003,
                 scoring_protocol='drawer_visible_pull_then_return_v1')
        facts=('RIGHT grips the existing upper-drawer edge, pulls it visibly outward, then pushes it back to its initial opening and releases. '
               'There is no prescribed 50 mm stroke. A net held pull of 5 mm is the numerical motion-detection threshold, not a real-task travel requirement. '
               'The upper drawer starts 30 mm open; return tolerance is 3 mm. LEFT observes. '
               'Five separate free 30 x 30 x 24-32 mm blocks (three blue, two green) sit inside the upper drawer, two near the opening and three farther inside; the lower drawer is empty. '
               'Cabinet and lower drawer are fixed fixtures; native passive upper rail is preserved. No added handle or object attachment. '
               'The cabinet is diagonal at the far right, partly outside the overview, matching the reference framing.')
    # Fit scene placement to the real first frame, using native sizes unchanged.
    from ...paths import PROJECT_ROOT
    from ...robots.transforms import pose_transform, transform_pose
    import numpy as np
    alignment=json.loads((PROJECT_ROOT/'assets/real2sim_l2_20261013/alignment.json').read_text())
    layout=alignment['layouts'].get(tid,{})
    if 'box_xy_yaw' in layout:
        x,y,yaw=layout['box_xy_yaw']
        old=p['pedestal_pose'] if tid=='L2-06' else p['native_instances'][0]['pose']
        new=pose(x,y,.78,yaw=yaw)
        delta=pose_transform(new)@np.linalg.inv(pose_transform(old))
        def moved(q):return transform_pose(delta@pose_transform(q))
        height=.10  # User confirmed shared 200 x 200 x 100 mm box, 2026-10-13
        dz=(height-.10)*(0.935 if tid!='L2-03' else 1.)
        def moved_article(q):
            v=list(moved(q));v[2]+=dz;return tuple(v)
        articles=[replace(a,initial=moved_article(a.initial),goal=moved_article(a.goal)) for a in articles]
        p['support_z']+=dz
        p['support_box_height_m']=height
        p['support_box_scale_z']=height/.10
        p['support_box_scale']=(.200/.190,1.,height/.10)
        p['support_box_dimensions_m']=(.200,.200,.100)
        p['dimension_evidence']='User confirmed shared box dimensions, 2026-10-13'
        p['placement_evidence']='Photo/video estimate authorized by user; not measured'
        p['fixation_evidence']='Simulation fixed; real fixation unconfirmed'
        if tid=='L2-06':p['support_half_extents_m']=(.092,.092)
        if tid=='L2-06':p['pedestal_pose']=new
        else:
            p['native_instances']=[dict(i,pose=moved(i['pose']),scale=p['support_box_scale']) if i['name']=='support_box' else dict(i,pose=moved_article(i['pose'])) for i in p['native_instances']]
            if 'outward_axis_world' in p:p['outward_axis_world']=tuple(delta[:3,:3]@np.array(p['outward_axis_world']))
    elif tid=='L2-04':
        oldcups={a.name:a for a in articles[:2]}
        shifts={name:np.array([*layout[name],oldcups[name].initial[2]])-np.array(oldcups[name].initial[:3]) for name in oldcups}
        def shifted(q,name):return tuple(np.array(q[:3])+shifts[name])+tuple(q[3:])
        articles=[replace(a,initial=shifted(a.initial,a.name if a.name in oldcups else 'source_cup'),
            goal=shifted(a.goal,a.name if a.name in oldcups else 'receiver_cup')) for a in articles]
        p['native_instances']=[dict(i,pose=next(a.initial for a in articles if a.name==i['name'])) for i in p['native_instances']]
    if tid=='L2-03':
        # User correction: move books toward the wall/arms (+world Y), not the camera.
        def toward_wall(q):return (q[0],q[1]+.025,*q[2:])
        articles=[replace(a,initial=toward_wall(a.initial),goal=toward_wall(a.goal)) for a in articles]
        p['native_instances']=[dict(i,pose=toward_wall(i['pose'])) if i['name']!='support_box' else i for i in p['native_instances']]
        p['book_wall_revision_m']=.025
        facts=facts.replace('20 mm toward the arms','45 mm toward the arms')
        facts+=' User revision: all three books moved 25 mm toward the wall/arms relative to the original aligned layout (50 mm from the reversed revision), with the box stationary.'
    if tid=='L2-07':
        shift=(-.020,-.120,.075)
        def translated(q):return tuple(q[j]+shift[j] for j in range(3))+tuple(q[3:])
        articles=[replace(a,initial=translated(a.initial),goal=translated(a.goal)) for a in articles]
        p['closed_pose']=translated(p['closed_pose'])
        p['native_instances']=[dict(i,pose=translated(i['pose'])) for i in p['native_instances']]
        cabinet=p['native_instances'][0]['pose']
        p['native_instances'].append(dict(name='support_box',asset='green_support_box',
            pose=(*cabinet[:2],.78,*cabinet[3:]),scale=(.280/.190,.160/.200,.75),static=[''],tracked={}))
        p['cabinet_support_height_m']=.075
        # Refit the cabinet against four visible drawer-rim landmarks; keep camera fixed.
        fitted=pose(.5024095451641413,.9300328413212409,.855,yaw=math.radians(-125.63513785056286))
        cabinet_delta=pose_transform(fitted)@np.linalg.inv(pose_transform(cabinet))
        def refitted(q):return transform_pose(cabinet_delta@pose_transform(q))
        articles=[replace(a,initial=refitted(a.initial),goal=refitted(a.goal)) for a in articles]
        p['closed_pose']=refitted(p['closed_pose'])
        p['outward_axis_world']=tuple(cabinet_delta[:3,:3]@np.asarray(p['outward_axis_world']))
        p['native_instances']=[dict(i,pose=refitted(i['pose'])) for i in p['native_instances']]
        p['drawer_blocks_revision']='two_front_three_occluded_32mm'
        facts+=' User revision: organizer rests on a teal-green support box (280 x 160 x 75 mm visual estimate). All five blocks remain loose in the upper drawer; taller appearance is not measured calibration.'
    p['canonical_scene_instruction']=True
    from .strict_l2 import PROTOCOL
    if tid in ('L2-01','L2-03'):
        p['strict_manipulation_protocol']=PROTOCOL
        p['scoring_protocol']=PROTOCOL
    if tid=='L2-03':
        p.pop('minimum_travel_m',None)
        p['complete_extraction_required']=True
    if tid=='L2-01':
        facts=('The capped marker lies horizontal on the shared 200 x 200 x 100 mm teal box. '
               'LEFT picks up the barrel with the cap still attached; lift both parts clear of the box and hold steadily. '
               'Then RIGHT grips the cap while LEFT retains the barrel and actively pulls axially toward image-right. '
               'Verify complete separation and keep both parts steadily held. A cap falling off during pickup is failure. '
               'No placement, release or return home is required. Native length 151 mm, barrel 16.4 mm, cap 17.6 mm. '
               'Both hands start empty; no object attachment. Detection uses 8 mm lift clearance, 0.25 s stable holds and 2 mm final axial gap; these are numerical tolerances, not measured real thresholds.')
    elif tid=='L2-03':
        facts=('Three estimated 220 x 140 x 20 mm books lie flat on the shared 200 x 200 x 100 mm teal box. '
               'The stack is offset to overview-image RIGHT and toward the wall/arms; its latest revision moved all books 25 mm toward the wall with the box stationary. '
               'The middle book initially protrudes approximately 40 mm to image-right. '
               'RIGHT grips the middle book across its thickness; LEFT steadies the upper book. '
               'Fully extract the middle book toward image-right until it no longer overlaps or contacts either neighbouring book. '
               'A 20 mm tug is NOT completion. Keep the upper and lower books stable on the box, allowing the upper book to settle naturally onto the lower book. '
               'Then place the extracted book safely on the table or box outside the remaining stack, release all books and withdraw. '
               'Do not drop books off the support. Numerical verification uses 2 mm separation clearance and 0.5 s final stability. '
               'Book dimensions, initial offsets and placements remain image estimates, not measurements.')
    elif tid=='L2-06':
        facts=('The rounded orange lies flat, long axis world X, on the shared 200 x 200 x 100 mm teal box. '
               'RIGHT grasps its fuller middle and lifts the orange 5-20 cm from its resting position, then returns it to the SAME box, releases and withdraws. '
               'LEFT observes only. Use gradual closure and inspect deformation; gripper commands specify opening, not force. '
               'The material is uncalibrated and has no tissue damage model; lack of simulated tearing is not proof of real-world safety.')
    if tid in ('L2-01','L2-03','L2-06'):
        facts+=' Box dimensions are user-confirmed. Position is estimated from reference images; the box is fixed in simulation, with real fixation still unconfirmed.'
    from ...backends.measured_table import revise_layout
    articles,p,facts=revise_layout(articles,p,facts,task,tid)
    p['real2sim_profile']=PROFILE
    return replace(spec,design=replace(spec.design,articles=tuple(articles),
        version=spec.design.version+('-measured-table-box-arms-v1' if task.get('table_revision') else '')+'-feishu-20261013-definition-v1'+('-books-wall-r3' if tid=='L2-03' else '-drawer-reference-r5' if tid=='L2-07' else ''),
        provenance='Feishu L2 video + user assets and 5/5 clarification; visual estimates, not metric calibration'),
        parameters=p,facts=facts,
        limitations=('User-confirmed shared box dimensions; other sizes, placement and material properties remain estimates. Dynamic qualification status is recorded per episode and does not establish real2sim calibration.'
                     if tid in ('L2-03','L2-04','L2-06','L2-07') else spec.limitations))

def apply(backend,task):
    from pxr import Gf,UsdLux,UsdShade,UsdGeom,Sdf
    import carb
    from robot_tasks.backends.appearance import apply_r5_appearance
    from .l1_01_detail import apply as detail
    stage=backend.world.stage
    report={'profile':PROFILE,'task':task.config.task_id,'reference':'Feishu L2 first-frame overview and wrist views',
            'calibration':'visual estimate, not surveyed camera/metric calibration'}
    report['appearance']={m.name:apply_r5_appearance(stage,m.prim_path) for m in task.config.robot.mounts}
    for mount in task.config.robot.mounts:
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(mount.prim_path+'/') and '/Appearance/Material_' in str(prim.GetPath()) and prim.IsA(UsdShade.Shader):
                shader=UsdShade.Shader(prim); color=shader.GetInput('diffuseColor').Get()
                if color and color[0]>.9 and color[1]<.01 and color[2]<.01:
                    shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.3,.8,1.))
    front=UsdLux.RectLight.Define(stage,'/World/Lights/L1VideoFrontFill')
    front.CreateIntensityAttr(1700);front.CreateNormalizeAttr(True)
    front.CreateWidthAttr(1.2);front.CreateHeightAttr(.8)
    xf=UsdGeom.Xformable(front);xf.AddTranslateOp().Set(Gf.Vec3d(.05,.3,1.7));xf.AddRotateXOp().Set(55.)
    # Preserve the reference episode's initial opposed wrist rolls.
    wrist_poses={str(p.GetPath()):UsdGeom.Xformable(p).GetLocalTransformation() for p in stage.Traverse() if str(p.GetPath()).split('/')[-1] in ('link6','link7','link8') and str(p.GetPath()).startswith('/World/Robots/')}
    report['robot_detail']=detail(stage,task.config.robot)
    if task.config.task_id=='L2-01':
        for path,matrix in wrist_poses.items(): UsdGeom.Xformable(stage.GetPrimAtPath(path)).MakeMatrixXform().Set(matrix)
    report['initial_wrist_pose']='Original opposed wrist rolls for L2-01; zero-roll inspection pose for other tasks; static authored only'
    oak=UsdShade.Shader(stage.GetPrimAtPath('/World/Materials/Oak/Texture'))
    oak.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.30,.34,.39,1.))
    oak.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.10,.10,.095,0.))
    UsdShade.Shader(stage.GetPrimAtPath('/World/Materials/Oak/Surface')).GetInput('roughness').Set(.72)
    cork=UsdShade.Shader(stage.GetPrimAtPath('/World/Materials/Floor/Texture'))
    cork.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.35,.42,.50,1.))
    cork.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.035,.030,.035,0.))
    UsdShade.Shader(stage.GetPrimAtPath('/World/Materials/Wall/Surface')).GetInput('diffuseColor').Set(Gf.Vec3f(.30,.40,.45))
    for name,intensity in [('Fill',1800),('Area0',5500),('Area1',4200)]:
        light=UsdLux.LightAPI(stage.GetPrimAtPath('/World/Lights/'+name))
        light.GetIntensityAttr().Set(intensity);light.GetColorAttr().Set(Gf.Vec3f(.88,.94,1.))
    front.CreateColorAttr(Gf.Vec3f(.84,.92,1.))
    if task.config.task_id=='L2-07':
        from ...paths import PROJECT_ROOT
        # Use a clean table-only window of the unmodified real reference as an
        # image-based diffuse texture. This includes captured illumination;
        # it is an appearance approximation, not a calibrated material BRDF.
        oak.GetInput('file').Set(Sdf.AssetPath(str(PROJECT_ROOT/'assets/real2sim_l2_20261013/reference_appearance/l207_initial_reference.png')))
        uv=UsdShade.Shader.Define(stage,'/World/Materials/Oak/ReferenceUV')
        uv.CreateIdAttr('UsdTransform2d')
        uv.CreateInput('in',Sdf.ValueTypeNames.Float2).ConnectToSource(UsdShade.Shader(stage.GetPrimAtPath('/World/Materials/Oak/ST')).ConnectableAPI(),'result')
        uv.CreateInput('scale',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(.60,.45))
        uv.CreateInput('translation',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(.08,.025))
        uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
        oak.GetInput('st').ConnectToSource(uv.ConnectableAPI(),'result')
        oak.GetInput('scale').Set(Gf.Vec4f(.85,.85,.85,1.))
        oak.GetInput('bias').Set(Gf.Vec4f(0,0,0,0))
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith('/World/L2/organizer/') and '/Contents/' not in str(prim.GetPath()) and prim.IsA(UsdShade.Shader):
                color=UsdShade.Shader(prim).GetInput('diffuseColor')
                if color and color.Get() and min(color.Get())>.5:
                    color.Set(Gf.Vec3f(.55,.85,.92))
        report['drawer_reference_revision']='Two partially visible front blocks; three under cover; source-photo table diffuse window; fixed observation camera'
    # Session-only colour matching; source delivery files remain hash-locked.
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith('/World/L2/') and path.endswith('/Looks/Teal/Texture'):
            UsdShade.Shader(prim).CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.,1.,1.,1.) if task.config.task_id=='L2-07' else Gf.Vec4f(.45,.80,1.10,1.))
        if task.config.task_id!='L2-04' and path.startswith('/World/L2/') and path.endswith(('/Looks/Interior/Surface','/Looks/Paper/Surface')):
            UsdShade.Shader(prim).GetInput('diffuseColor').Set(Gf.Vec3f(.94,.97,1.))
    from .l2_visual_detail import apply as visual_detail
    report["photo_detail"]=visual_detail(stage,task.config.task_id)
    settings=carb.settings.get_settings()
    settings.set_bool('/rtx/indirectDiffuse/enabled',True)
    settings.set_bool('/rtx/post/histogram/enabled',False)
    settings.set_string('/rtx/rendermode','PathTracing')
    settings.set_int('/rtx/pathtracing/spp',32);settings.set_int('/rtx/pathtracing/totalSpp',128)
    settings.set_bool('/rtx/pathtracing/optixDenoiser/enabled',True)
    settings.set_int('/rtx/pathtracing/maxBounces',8)
    settings.set_int('/rtx/pathtracing/maxSpecularAndTransmissionBounces',16)
    return report
