"""Open the portable Tuojing scene in Isaac Sim 5.1; optionally run a bounded smoke test."""
import argparse, os, json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--steps',type=int,default=0);p.add_argument('--demo',action='store_true');p.add_argument('--cutaway',action='store_true');p.add_argument('--scan',action='store_true');p.add_argument('--camera',default='ThirdPerson');p.add_argument('--capture',action='store_true');p.add_argument('--hfov',type=float);p.add_argument('--scene',default='tuojing_lab.usda');p.add_argument('--video-points',action='store_true');args=p.parse_args()
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES');os.environ.setdefault('OMNI_KIT_ALLOW_ROOT','1')
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'width':1440,'height':1080,'renderer':'RaytracedLighting','extra_args':['--/app/renderer/skipDriverVersionCheck=true']})
import omni.usd
import numpy as np
from pxr import UsdGeom,UsdPhysics,Usd,UsdUtils
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'layout_parameters.json').read_text())
camera_path={'WristLeft':'/World/Robots/R5_Left/link6/WristSensor/Sensor/Camera','WristRight':'/World/Robots/R5_Right/link6/WristSensor/Sensor/Camera'}.get(args.camera,'/World/Cameras/'+args.camera)
omni.usd.get_context().open_stage(str(ROOT/args.scene))
for _ in range(20):app.update()
stage=omni.usd.get_context().get_stage()
# Changes made by this launcher are session-local and do not modify the saved asset.
stage.SetEditTarget(stage.GetSessionLayer())
if args.cutaway:
 for path in ['/World/Room/Ceiling','/World/Room/DeskWall','/World/Room/CorkPanel','/World/Room/PowerStrip']+[f'/World/Room/WallPanel_{i}' for i in range(5)]+[f'/World/Room/PanelTrim{i}' for i in range(6)]+[f'/World/Room/Socket{i}' for i in range(6)]:
  if stage.GetPrimAtPath(path).IsValid():UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()
if not args.headless:
 from omni.kit.viewport.utility import get_active_viewport
 vp=get_active_viewport()
 if vp:vp.set_active_camera('/World/Cameras/Overview' if args.cutaway else camera_path)
sim=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
robots=[sim.scene.add(Robot(prim_path='/World/Robots/'+name+'/root_joint',name=name)) for name in ['R5_Left','R5_Right']]
sim.reset()
qs=[np.array(rc['home_q_rad_m'],dtype=np.float32) for rc in P['robots']]
for robot,q in zip(robots,qs):robot.set_joint_positions(q);robot.set_joints_default_state(positions=q);robot.apply_action(ArticulationAction(joint_positions=q))
if args.hfov:
 import math
 camera=UsdGeom.Camera(stage.GetPrimAtPath(camera_path));camera.GetFocalLengthAttr().Set(float(camera.GetHorizontalApertureAttr().Get())/(2*math.tan(math.radians(args.hfov/2))))
if args.scan:
 for path in ['/World/Room','/World/Furniture','/World/Robots']:UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()
 UsdGeom.Imageable(stage.GetPrimAtPath('/World/ReferenceScan')).MakeVisible()
if args.video_points:UsdGeom.Imageable(stage.GetPrimAtPath('/World/LatestVideoPoints')).MakeVisible()
i=0
try:
 while app.is_running() and (not args.steps or i<args.steps):
  for side,robot in enumerate(robots):
   target=qs[side].copy()
   if args.demo:target[0]=(-1 if side==0 else 1)*.12*np.sin(i/60.)
   robot.apply_action(ArticulationAction(joint_positions=target))
  sim.step(render=True);i+=1
 if args.steps:
  a=np.array([r.get_joint_positions() for r in robots]);assert a.shape==(2,8) and np.isfinite(a).all()
  cache=UsdGeom.XformCache();props={str(x.GetName()):list(cache.GetLocalToWorldTransform(x).ExtractTranslation()) for x in stage.GetPrimAtPath('/World/Props').GetChildren()}
  assert len(robots)==2
  _,_,missing=UsdUtils.ComputeAllDependencies(str(ROOT/args.scene))
  assert not [str(x) for x in missing if not str(x).endswith('.mdl')]
  result={'fresh_process_load':True,'dual_articulations':2,'steps':i,'finite_joint_state':True,'joint_positions':a.tolist(),'props_xyz':props,'missing_dependencies':[str(x) for x in missing]}
  (ROOT/'reopen_validation.json').write_text(json.dumps(result,indent=2));print('REOPEN_PASS',json.dumps(result),flush=True)
 if args.capture:
  import omni.replicator.core as rep
  from PIL import Image
  from pxr import Gf
  destination=ROOT/'observations';destination.mkdir(exist_ok=True)
  path=camera_path;cam=UsdGeom.Camera(stage.GetPrimAtPath(path));res=P['camera']['resolution']
  product=rep.create.render_product(path,res);rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach(product);depth.attach(product)
  for _ in range(25):app.update()
  rep.orchestrator.step(rt_subframes=16,delta_time=0,pause_timeline=True)
  pixels=rgb.get_data();distance=depth.get_data();assert pixels.size and distance.size
  valid=np.isfinite(distance)&(distance>0);assert valid.mean()>.8
  suffix='' if args.hfov is None else '_hfov'+str(int(args.hfov));Image.fromarray(pixels).save(destination/(args.camera+suffix+'_rgb.png'));np.save(destination/(args.camera+suffix+'_depth_m.npy'),distance)
  cache=UsdGeom.XformCache();M=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T@np.diag([1,-1,-1,1]);fx=res[0]*cam.GetFocalLengthAttr().Get()/cam.GetHorizontalApertureAttr().Get()
  metadata={'scene':args.scene,'joint_positions':{r.name:r.get_joint_positions().tolist() for r in robots},'camera':path,'resolution':res,'K':[[float(fx),0,res[0]/2],[0,float(fx),res[1]/2],[0,0,1]],'T_world_from_optical':M.tolist(),'depth_definition':'optical Z in metres; not Euclidean range','valid_depth_fraction':float(valid.mean()),'depth_median_m':float(np.median(distance[valid])),'distortion_calibrated':False}
  (destination/(args.camera+suffix+'_metadata.json')).write_text(json.dumps(metadata,indent=2));print('CAPTURE_PASS',json.dumps(metadata),flush=True);rgb.detach(product);depth.detach(product);product.destroy()
finally:app.close()
