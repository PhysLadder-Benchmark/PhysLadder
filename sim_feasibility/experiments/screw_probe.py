"""Probe RoboDojo nut/bolt contact under axial loading and signed torque.

A coaxial fixture removes perception and alignment uncertainty. This measures
an asset mechanism, not autonomous screwing, stripping, or real-world torque.
"""
import argparse
import csv
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
project = Path(__file__).resolve().parents[1]
assets = project / 'assets/robodojo'

from isaacsim import SimulationApp
from runtime_config import simulation_config
app = SimulationApp(simulation_config())

import numpy as np
import warp as wp
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Usd, UsdShade
from PIL import Image

dt, duration = 1/1200, 4.0
world = World(stage_units_in_meters=1, physics_dt=dt, rendering_dt=1/60, backend='warp', device='cuda')
stage = world.stage
world.get_physics_context().set_solver_type('TGS')
world.get_physics_context().enable_gpu_dynamics(True)
world.get_physics_context().set_broadphase_type('GPU')
floor = UsdGeom.Cube.Define(stage, '/World/Floor')
floor.CreateSizeAttr(1)
floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-0.01))
floor.AddScaleOp().Set(Gf.Vec3f(2,2,0.02))
UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000)
material = UsdShade.Material.Define(stage, '/World/ThreadMaterial')
mat_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
mat_api.CreateStaticFrictionAttr(0.15)
mat_api.CreateDynamicFrictionAttr(0.15)
mat_api.CreateRestitutionAttr(0.0)

def place(prim, position):
    # Author before simulation using USD; avoid CPU/CUDA frontend conversions.
    transform = UsdGeom.Xformable(prim)
    transform.MakeMatrixXform().Set(Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(*position)))
    scale = prim.GetAttribute('xformOp:scale')
    if scale: scale.Set((1.0,1.0,1.0))

cases = []
for i, (speed, bolt_collision) in enumerate([(0.0,True), (2.0,True), (-2.0,True), (-2.0,False)]):
    x = (i-1.5)*0.12
    root = f'/World/Case_{i}'
    bolt = add_reference_to_stage(str(assets/'Geometry/factory_bolt/00000/object.usdz'), root+'/Bolt')
    UsdPhysics.RigidBodyAPI(bolt).CreateRigidBodyEnabledAttr(False)
    # Replace only the root pose. The internal asset geometry is unchanged.
    # The source bolt has a dormant xformOp:scale=1.5 which XFormPrim would
    # otherwise activate while rebuilding xformOpOrder. Set scale explicitly.
    place(bolt, (x,0,0.05))
    nut = add_reference_to_stage(str(assets/'Rigid/factory_nut/00000/object.usdz'), root+'/Nut')
    place(nut, (x,0,0.115))
    UsdPhysics.MassAPI.Apply(nut).CreateMassAttr(0.03)
    body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(nut)
    body_api.CreateSolverPositionIterationCountAttr(64)
    body_api.CreateSolverVelocityIterationCountAttr(16)
    # USD angular limits use degrees/s; tensor angular velocities use rad/s.
    body_api.CreateMaxAngularVelocityAttr(float(np.degrees(5.0)))
    body_api.CreateMaxDepenetrationVelocityAttr(0.2)
    if not bolt_collision:
        body_api.CreateDisableGravityAttr(True)
    for parent in [bolt, nut]:
        for prim in Usd.PrimRange(parent):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if parent == bolt:
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(bolt_collision)
                collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collision.CreateContactOffsetAttr(0.0001)
                collision.CreateRestOffsetAttr(0.0)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    # Lock lateral translation and tilt; leave axial translation and rotation
    # physically free. The fixture itself does not impose a screw pitch.
    joint = UsdPhysics.Joint.Define(stage, root+'/Fixture')
    joint.CreateBody1Rel().SetTargets([Sdf.Path(root+'/Nut')])
    joint.CreateLocalPos0Attr(Gf.Vec3f(x, 0, 0.115))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    for axis in ['transX', 'transY', 'rotX', 'rotY']:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        # USD uses low > high for LOCKED. Equal zero limits create an
        # unsupported double-pyramid D6 configuration in this PhysX version.
        limit.CreateLowAttr(1)
        limit.CreateHighAttr(-1)
    cases.append({'id':i, 'target_speed_rad_s':speed, 'force_z_N':-0.2 if bolt_collision else 0.0,
                  'bolt_collision':bolt_collision})

view = world.scene.add(RigidPrim('/World/Case_.*/Nut', name='nuts', track_contact_forces=True, reset_xform_properties=False))
camera = rep.create.camera(position=(0.25, -0.45, 0.28), look_at=(0, 0, 0.09), clipping_range=(0.005, 100))
product = rep.create.render_product(camera, (960, 720))
rgb = rep.AnnotatorRegistry.get_annotator('rgb')
rgb.attach(product)
rep.orchestrator.set_capture_on_play(False)
world.reset()
case_ids = [int(p.split('Case_')[1].split('/')[0]) for p in view.prim_paths]
stage.GetRootLayer().Export(str(a.output/'initial_scene.usda'))
rows, frames = [], []
for step in range(round(duration/dt)):
    forces = np.array([[0,0,cases[i]['force_z_N']] for i in case_ids],dtype=np.float32)
    omega = view.get_angular_velocities().numpy()
    torques = np.zeros_like(forces)
    for j,i in enumerate(case_ids):
        torques[j,2] = np.clip(0.002*(cases[i]['target_speed_rad_s']-omega[j,2]), -0.01, 0.01)
    view.apply_forces_and_torques_at_pos(
        forces=wp.array(forces,dtype=wp.float32,device='cuda:0'),
        torques=wp.array(torques,dtype=wp.float32,device='cuda:0'))
    world.step(render=False)
    pos, quat = view.get_world_poses()
    pos, quat = pos.numpy(), quat.numpy()
    angular = view.get_angular_velocities().numpy()
    contact = view.get_net_contact_forces(dt=dt).numpy()
    for j, i in enumerate(case_ids):
        rows.append({'time_s':(step+1)*dt,'case_id':i,'z':float(pos[j,2]),
                     'angular_velocity_z':float(angular[j,2]),
                     'applied_torque_z_Nm':float(torques[j,2]),
                     'quat_w':float(quat[j,0]),'quat_z':float(quat[j,3]),
                     'contact_fz':float(contact[j,2])})
    if step%60==0:
        world.render()
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=2)
        pixels=rgb.get_data()
        if pixels is not None and np.asarray(pixels).size:
            frames.append(Image.fromarray(np.asarray(pixels)[...,:3].astype(np.uint8)))
with (a.output/'trajectory.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary=[]
for c in cases:
    rr=[r for r in rows if r['case_id']==c['id']]
    summary.append({**c,'final_z_m':float(np.mean([r['z'] for r in rr[-120:]])),
                    'integrated_rotation_rad':sum(r['angular_velocity_z']*dt for r in rr)})
rotation_depth_effect=max(abs(c['final_z_m']-summary[0]['final_z_m']) for c in summary[1:] if c['bolt_collision'])
result={'physics_dt':dt,'duration_s':duration,'cases':summary, 'rendered_frames':len(frames),
        'friction':0.15,'speed_controller_gain_Nms':0.002,'max_torque_Nm':0.01,
        'physics_backend':'GPU PhysX / Warp',
        'rotation_control_passed':abs(summary[3]['integrated_rotation_rad'])>3,
        'rotation_changes_depth_by_m':rotation_depth_effect,
        'interpretation':'measurement only; assess trajectory and geometry before concluding thread engagement',
        'scope':'coaxial object fixture, no robot/GPT/R5; no stripping or damage model'}
(a.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
if frames:
    frames[0].save(a.output/'first_frame.png')
    frames[-1].save(a.output/'last_frame.png')
    frames[0].save(a.output/'rollout.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
print('SCREW_PROBE_RESULT '+json.dumps(result),flush=True)
app.close()
