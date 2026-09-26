"""Procedural FEM compression, not a calibrated bread or fracture model.

Two identical elastic blocks differ only in Young's modulus. Dynamic pistons
apply the same force and then lift away. No deformable nodes are teleported.
"""
import argparse
import csv
import json
from pathlib import Path
from runtime_config import simulation_config

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp(simulation_config())

import numpy as np
import warp as wp
import omni.physics.tensors as tensors
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.deformable_mesh_utils import createTriangleMeshCube
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
from PIL import Image

dt = 1 / 240
world = World(stage_units_in_meters=1, physics_dt=dt, rendering_dt=1/60,
              backend='warp', device='cuda')
# PhysX and its tensor frontend both use GPU through bundled NVIDIA Warp.
# No CUDA build of PyTorch is needed for this mechanism-only experiment.
world.get_physics_context().enable_gpu_dynamics(True)
world.get_physics_context().set_broadphase_type('GPU')
stage = world.stage
UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(900)

def cube(path, position, size, mass=None):
    obj = UsdGeom.Cube.Define(stage, path)
    obj.CreateSizeAttr(1)
    obj.AddTranslateOp().Set(Gf.Vec3d(*position))
    obj.AddScaleOp().Set(Gf.Vec3f(*size))
    obj.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.5, 0.7)])
    prim = obj.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(0.0005)
    collision.CreateRestOffsetAttr(0)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
    return prim

cube('/World/Floor', (0, 0, -0.01), (0.6, 0.4, 0.02))
cases = []
for i, young in enumerate([10000.0, 100000.0]):
    x, root = (i-0.5)*0.18, f'/World/Case_{i}'
    points, indices = createTriangleMeshCube(6)
    pts = np.asarray(points, dtype=float)
    pts = (pts - (pts.min(0) + pts.max(0))/2) / np.ptp(pts, axis=0)
    pts *= np.array([0.06, 0.06, 0.06])
    mesh = UsdGeom.Mesh.Define(stage, root+'/Soft')
    mesh.CreatePointsAttr(pts.tolist())
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateFaceVertexCountsAttr([3]*(len(indices)//3))
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.AddTranslateOp().Set(Gf.Vec3d(x, 0, 0.0305))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.25+i*0.25, 0.15)])
    deformableUtils.add_deformable_body_material(stage, root+'/Material', youngs_modulus=young,
                                                poissons_ratio=0.3, dynamic_friction=0.3,
                                                damping_scale=0.05, elasticity_damping=0.01)
    deformableUtils.add_physx_deformable_body(stage, root+'/Soft',
                                            simulation_hexahedral_resolution=8,
                                            solver_position_iteration_count=32,
                                            vertex_velocity_damping=1.0, self_collision=False)
    physicsUtils.add_physics_material_to_prim(stage, mesh.GetPrim(), root+'/Material')
    soft_collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    soft_collision.CreateContactOffsetAttr(0.0005)
    soft_collision.CreateRestOffsetAttr(0.0)
    UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateDensityAttr(200.0)
    piston = cube(root+'/Piston', (x,0,0.067), (0.08,0.08,0.01), mass=0.05)
    joint = UsdPhysics.PrismaticJoint.Define(stage, root+'/Guide')
    joint.CreateBody1Rel().SetTargets([Sdf.Path(root+'/Piston')])
    joint.CreateLocalPos0Attr(Gf.Vec3f(x,0,0.067))
    joint.CreateAxisAttr('Z')
    joint.CreateLowerLimitAttr(-0.05)
    joint.CreateUpperLimitAttr(0.08)
    cases.append({'id':i, 'youngs_modulus_Pa':young})

pistons = world.scene.add(RigidPrim('/World/Case_.*/Piston', name='pistons'))
camera = rep.create.camera(position=(0.32,-0.5,0.28), look_at=(0,0,0.04), clipping_range=(0.005,100))
product = rep.create.render_product(camera, (1024,768))
rgb = rep.AnnotatorRegistry.get_annotator('rgb')
rgb.attach(product)
rep.orchestrator.set_capture_on_play(False)
world.reset()
tensor_view = tensors.create_simulation_view('warp')
tensor_view.set_subspace_roots('/')
soft_views = [tensor_view.create_soft_body_view(f'/World/Case_{c["id"]}/Soft') for c in cases]
assert all(v.count == 1 for v in soft_views)
stage.GetRootLayer().Export(str(a.output/'initial_scene.usda'))
case_ids = [c['id'] for c in cases]
piston_ids = [int(path.split('Case_')[1].split('/')[0]) for path in pistons.prim_paths]
assert piston_ids == case_ids
rows, frames, snapshots = [], [], {}
for step in range(720):
    t = step*dt
    force = -2.0 if 0.5 <= t < 1.6 else (2.0 if 1.6 <= t < 2.1 else 0.0)
    forces = np.zeros((2,3), dtype=np.float32)
    forces[:,2] = force
    pistons.apply_forces(wp.array(forces, dtype=wp.float32, device='cuda:0'))
    world.step(render=False)
    nodes = np.stack([v.get_sim_nodal_positions().numpy()[0] for v in soft_views])
    positions, _ = pistons.get_world_poses()
    positions = positions.numpy()
    for j, case_id in enumerate(case_ids):
        z = nodes[j,:,2]
        rows.append({'time_s':(step+1)*dt, 'case_id':case_id,
                     'height_m':float(np.max(z)-np.min(z)),
                     'min_z_m':float(np.min(z)), 'max_z_m':float(np.max(z)),
                     'piston_z_m':float(positions[j,2]), 'applied_force_N':force,
                     'finite':bool(np.isfinite(nodes[j]).all())})
    if step in [119, 359, 719]:
        snapshots[str(step)] = nodes.copy()
    if step % 12 == 0:
        # GPU physics writes Fabric; synchronize it before the camera reads it.
        world.render()
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=2)
        pixels = rgb.get_data()
        if pixels is not None and np.asarray(pixels).size:
            frames.append(Image.fromarray(np.asarray(pixels)[...,:3].astype(np.uint8)))
with (a.output/'trajectory.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
np.savez_compressed(a.output/'nodal_snapshots.npz', **snapshots)
checks=[]
for case in cases:
    rr=[r for r in rows if r['case_id']==case['id']]
    mean=lambda start,end:float(np.mean([r['height_m'] for r in rr if start<=r['time_s']<end]))
    base,loaded,unloaded=mean(.35,.49),mean(1.3,1.59),mean(2.7,3.01)
    checks.append({**case,'baseline_height_m':base,'loaded_height_m':loaded,
                   'unloaded_height_m':unloaded,'compression_m':base-loaded,
                   'residual_m':base-unloaded,'finite':all(r['finite'] for r in rr)})
passed=(all(c['finite'] for c in checks)
        and checks[0]['compression_m'] > 2*checks[1]['compression_m'] > 0.0001
        and all(abs(c['residual_m'])<0.001 for c in checks) and len(frames)>5)
result={'cases':checks,'passed':bool(passed),'rendered_frames':len(frames),
        'physics_dt':dt,'force_N':2.0,'frontend':'warp GPU',
        'scope':'procedural elastic FEM blocks only; no bread assets, plasticity, fracture or robot'}
(a.output/'summary.json').write_text(json.dumps(result,indent=2))
if frames:
    frames[0].save(a.output/'first_frame.png');frames[-1].save(a.output/'last_frame.png')
    frames[0].save(a.output/'rollout.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
print('DEFORMABLE_RESULT '+json.dumps(result),flush=True)
app.close()
raise SystemExit(0 if passed else 2)
