"""Opt-in measured L2 table; immutable delivery and legacy profiles retained."""
from dataclasses import replace
REVISION='159x79x88_20261013'
OLD_TOP=.78
TOP=.88
LIFT=TOP-OLD_TOP
BACK=1.637650417122201
CENTRE=(.04354960845762923,BACK-.79/2,TOP-.025/2)
SIZE=(1.59,.79,.025)
# Keep the existing drawer support footprint fully on the shorter table, 10 mm
# inside the front edge. All pieces move together; no rescaling of props.
DRAWER_SHIFT_Y=(BACK-.79)+.010-.7740086771886984
BOX_SIZE=(.205,.195,.075)

def enabled(config):
    return config.raw.get('task',{}).get('table_revision')==REVISION

def revise_layout(articles,parameters,facts,task,tid):
    if task.get('table_revision')!=REVISION:return articles,parameters,facts
    if task.get('real2sim_profile')!='feishu_l2_20261013':raise ValueError('Measured table requires aligned L2')
    dy=DRAWER_SHIFT_Y if tid=='L2-07' else 0.
    p=dict(parameters)
    shared=tid in ('L2-01','L2-03','L2-06')
    object_dz=(-.025 if tid=='L2-03' else -.025*.935) if shared else 0.
    def shift(q,dz=0.):return (q[0],q[1]+dy,q[2]+LIFT+dz,*q[3:])
    articles=[replace(a,initial=shift(a.initial,object_dz),goal=shift(a.goal,object_dz)) for a in articles]
    p['table_z']+=LIFT
    if 'support_z' in p:p['support_z']+=LIFT+object_dz
    for key in ['pedestal_pose','closed_pose']:
        if key in p:p[key]=shift(p[key])
    if 'native_instances' in p:
        p['native_instances']=[dict(i,pose=shift(i['pose'],0. if i['name']=='support_box' else object_dz),
             **({'scale':(.205/.190,.195/.200,.75)} if i['name']=='support_box' else {})) for i in p['native_instances']]
    if shared or tid=='L2-07':
        p['support_box_dimensions_m']=BOX_SIZE;p['support_box_height_m']=.075
        p['support_box_scale_z']=.75;p['support_box_scale']=(.205/.190,.195/.200,.75)
        p['dimension_evidence']='Latest user measurement: 205 x 195 x 75 mm, 2026-10-13; supersedes 200 x 200 x 100 mm estimate'
        if tid=='L2-06':p['support_half_extents_m']=(.087*.205/.190,.092*.195/.200)
        facts=facts.replace('200 x 200 x 100 mm','205 x 195 x 75 mm').replace('10 cm box','7.5 cm box').replace('280 x 160 x 75 mm visual estimate','205 x 195 x 75 mm user measurement')

    p['table_revision']=REVISION;p['table_dimensions_m']=(1.59,.79,.88);p['layout_translation_m']=(0.,dy,LIFT)
    facts+=' Table dimensions are user-confirmed: 1590 x 790 mm, surface 880 mm above the floor; table thickness retained at 25 mm (unmeasured).'
    if dy:facts+=' The complete drawer setup is moved {:.1f} mm toward the arms to keep the complete setup 10 mm inside the shortened table front.'.format(dy*1000)
    return articles,p,facts

def apply(stage,config):
    if not enabled(config):return {}
    import numpy as np
    from pxr import Gf,UsdGeom,UsdPhysics
    root=stage.GetPrimAtPath('/World/Furniture/Worktable')
    if root.GetCustomDataByKey('table_revision')==REVISION:
        return {'table_surface_z_m':TOP,'table_dimensions_m':[1.59,.79,.88]}
    def world(p):return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(p)).T
    def set_world(p,m):
        parent=np.asarray(UsdGeom.XformCache().GetParentToWorldTransform(p)).T
        UsdGeom.Xformable(p).MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent)@m).T.tolist()))
    def move(p,dy=0.,dz=0.):
        m=world(p);m[1,3]+=dy;m[2,3]+=dz;set_world(p,m)
    for p in root.GetChildren():
        n=p.GetName()
        if n=='Top':
            m=world(p);a=np.eye(4);a[0,0]=1.59/1.7;a[1,1]=.79/.9;a[0,3]=CENTRE[0]*(1-a[0,0]);a[1,3]=BACK*(1-a[1,1]);a[2,3]=LIFT;set_world(p,a@m)
        elif n.startswith('LegInner'):
            b=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(p).ComputeAlignedRange();low,high=b.GetMin()[2],b.GetMax()[2];a=np.eye(4);a[2,2]=(high-low+LIFT)/(high-low);a[2,3]=low*(1-a[2,2]);a[1,3]=.055;set_world(p,a@world(p))
        elif n.startswith(('TopSupport','Crossbar')):move(p,.055,LIFT)
        elif n=='Control':move(p,.11,LIFT)
        else:move(p,.055,0.)
    for mount in config.robot.mounts:
        robot=stage.GetPrimAtPath(mount.prim_path);m=world(robot)
        delta=np.asarray(mount.base_pose_wxyz[:3])-m[:3,3]
        m[:3,3]+=delta;set_world(robot,m)
        plate=stage.GetPrimAtPath('/World/Furniture/R5_'+mount.name.title()+'_Mount');m=world(plate);m[:3,3]+=delta;set_world(plate,m)
    root.SetCustomDataByKey('table_revision',REVISION);root.SetCustomDataByKey('table_lift_m',LIFT)
    # Require the actual colliding tabletop to match the public geometry.
    top=stage.GetPrimAtPath('/World/Furniture/Worktable/Top');b=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(top).ComputeAlignedRange()
    assert np.allclose(np.asarray(b.GetMax())-b.GetMin(),SIZE,atol=1e-6)
    assert abs(b.GetMax()[2]-TOP)<1e-6
    assert top.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(top).GetCollisionEnabledAttr().Get()
    return {'table_surface_z_m':TOP,'table_dimensions_m':[1.59,.79,.88],
            'measured_table_revision':REVISION,'table_anchor':'back edge and horizontal centre retained',
            'tabletop_thickness_m':.025,'tabletop_thickness_evidence':'retained prior estimate',
            'robots_and_mounts_lifted_m':LIFT,'arm_edge_distances_m':[.685,.470,.435],'arm_spacing_m':.470,'overview_camera_moved':False,'source_layers_modified':False}
