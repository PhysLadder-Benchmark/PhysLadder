from pathlib import Path
import sys,json,os,argparse
p=argparse.ArgumentParser();p.add_argument('task');p.add_argument('--mode',default='static');p.add_argument('--part',type=int,default=0);p.add_argument('--parts',type=int,default=1);args=p.parse_args();P=Path(__file__).resolve().parent;O=P/args.task/args.mode
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'limit_cpu_threads':8,'multi_gpu':False,'fast_shutdown':True})
import omni.usd,omni.timeline,carb,numpy as np,omni.replicator.core as rep
from pxr import Usd,UsdGeom,Gf
from PIL import Image
omni.usd.get_context().open_stage(str(O/'scene.usdc'));s=omni.usd.get_context().get_stage();timeline=omni.timeline.get_timeline_interface();timeline.stop();timeline.set_auto_update(False)
c=carb.settings.get_settings();c.set_string('/rtx/rendermode','PathTracing');c.set_bool('/rtx/post/histogram/enabled',False);c.set_int('/rtx/pathtracing/spp',8 if args.mode!='static' else 32);c.set_int('/rtx/pathtracing/totalSpp',16 if args.mode!='static' else 128);c.set_bool('/rtx/pathtracing/optixDenoiser/enabled',True);c.set_int('/rtx/pathtracing/maxBounces',8)
prods={};cameras=[('overview','/World/Cameras/ThirdPerson')]
if s.GetPrimAtPath('/World/Cameras/HandDetail'):cameras.append(('detail','/World/Cameras/HandDetail'))
elif s.GetPrimAtPath('/World/Cameras/InsertionDetail'):cameras.append(('detail','/World/Cameras/InsertionDetail'))
for n,path in cameras:
 product=rep.create.render_product(path,(960,720) if args.mode!='static' else (1280,960));a=rep.AnnotatorRegistry.get_annotator('rgb');a.attach(product);prods[n]=a
if args.mode=='static':
 for _ in range(35):app.update()
 for _ in range(3):rep.orchestrator.step(rt_subframes=8,delta_time=0,pause_timeline=True)
 for n,a in prods.items():Image.fromarray(np.array(a.get_data())[:,:,:3]).save(O/(n+'.png'))
 print('STATIC_RENDER_DONE',args.task,flush=True)
else:
 import imageio.v2 as imageio
 d=np.load(O/'motion.npz');paths=d['paths'];poses=d['transforms'];times=d['times'];ops=[];inverses=[];scales=[]
 G=UsdGeom.Xformable(s.GetPrimAtPath('/World/Insertion')).ComputeLocalToWorldTransform(0) if args.task=='L1-03' else Gf.Matrix4d(1)
 for path in paths:
  path=str(path).replace('/World/','/World/Insertion/',1) if args.task=='L1-03' else str(path)
  pr=s.GetPrimAtPath(path);xf=UsdGeom.Xformable(pr);scales.append(np.linalg.norm(np.array(xf.ComputeLocalToWorldTransform(0))[:3,:3],axis=1) if args.task in ['L3-03','L3-05'] else np.ones(3));inverses.append(UsdGeom.Xformable(pr.GetParent()).ComputeLocalToWorldTransform(0).GetInverse());xf.ClearXformOpOrder();ops.append(xf.AddTransformOp(opSuffix='recorded'))
 soft=np.load(O/'soft.npz') if (O/'soft.npz').exists() else None
 wanted=json.loads((O/'frame_times.json').read_text());indices=list(np.array_split(np.array([int(np.argmin(abs(times-t))) for t in wanted]),args.parts)[args.part]);writers={n:imageio.get_writer(str(O/f'{n}.part{args.part}.mp4'),fps=15,codec='libx264',quality=8,macro_block_size=16,ffmpeg_params=['-pix_fmt','yuv420p','-movflags','+faststart']) for n in prods}
 for _ in range(20):app.update()
 for i in indices:
  for j,tf in enumerate(poses[i]):
   m=Gf.Matrix4d().SetRotate(Gf.Quatd(float(tf[6]),Gf.Vec3d(*map(float,tf[3:6]))));m.SetTranslateOnly(Gf.Vec3d(*map(float,tf[:3])));ops[j].Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*map(float,scales[j])))*m*G*inverses[j])
  if soft is not None:UsdGeom.Mesh(s.GetPrimAtPath('/World/L2/orange/Visual')).GetPointsAttr().Set(soft['points'][i])
  app.update();rep.orchestrator.step(rt_subframes=2,delta_time=0,pause_timeline=True)
  for n,a in prods.items():
   data=np.array(a.get_data())[:,:,:3];writers[n].append_data(data)
   if i in [0,90,180,300]:Image.fromarray(data).save(O/f'{n}_{times[i]:05.2f}.png')
  if i%30==0:print('FRAME',args.task,i,float(times[i]),flush=True)
 for w in writers.values():w.close()
 (O/f'times.part{args.part}.json').write_text(json.dumps([float(times[i]) for i in indices]));print('VIDEO_RENDER_DONE',args.task,flush=True)
os._exit(0)
