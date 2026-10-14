from pathlib import Path
import json,numpy as np,os
from pxr import Usd,UsdGeom,Gf
from build_revision import build,P,ROOT
runs={'L1-01':'l101_sharpa_feasibility_20261014/trial03','L1-03':'l103_sharpa_insertion_20261014/video02','L1-07':'l1_batch_sharpa_20261014/07/video02','L1-08':'l1_batch_sharpa_20261014/08/video03','L1-09':'l109_contact_fix_20261014/video08','L2-01':'l2_sharpa_20261014/01/video03','L2-03':'l2_sharpa_20261014/03/video07','L2-04':'l2_sharpa_20261014/04/video02_fix','L2-06':'l2_sharpa_20261014/06/video05','L2-07':'l2_sharpa_20261014/07/video01','L3-03':'l303_material_fix_20261014/03/material01','L3-05':'l3_finger_twist_20261014/05/fingers04','L3-14':'l3_sharpa_20261014/14/trial05','L3-15':'l3_sharpa_20261014/15/trial05'}
for tid,rel in runs.items():
 src=ROOT/'reports'/rel;build(tid,src/('render-scene.usda' if tid=='L1-03' else 'scene.usda'),'video');out=P/tid/'video'
 for f in ['motion.npz','soft.npz','physics.npz','tactile.npz','taxel_projection.npz','thread_motion.npz','recording.json']:
  if (src/f).exists() and not (out/f).exists():(out/f).symlink_to(src/f)
 if (src/'motion_slow.npz').exists():
  a=np.load(src/'motion.npz');b=np.load(src/'motion_slow.npz');tt=np.r_[a['times'],b['times']];pp=np.concatenate([a['transforms'],b['transforms']]);order=np.argsort(tt,kind='stable');tt=tt[order];pp=pp[order];keep=np.r_[np.diff(tt)>1e-8,True]
  (out/'motion.npz').unlink();np.savez_compressed(out/'motion.npz',paths=a['paths'],times=tt[keep],transforms=pp[keep])
 if not (out/'frame_times.json').exists():
  d=np.load(src/'motion.npz');(out/'frame_times.json').write_text(json.dumps(d['times'][::2].tolist()))
 # Preserve the camera actually used by the published renderer, not a stale base-stage camera.
 s=Usd.Stage.Open(str(out/'scene.usdc'));c=UsdGeom.Camera(s.GetPrimAtPath('/World/Cameras/HandDetail'))
 if tid=='L1-09':eye=[.60,.65,1.57];target=[.18,1.2,1.02];focal=23
 elif tid=='L2-07':eye=[.07,1.35,1.32];target=[.44,1,1.04];focal=30
 elif tid in ['L3-03','L3-05']:
  d=np.load(src/'motion.npz');part='/Cap' if tid=='L3-03' else '/Nut';j=next(i for i,p in enumerate(d['paths']) if str(p).endswith(part));target=d['transforms'][min(120,len(d['times'])-1),j,:3]+[0,0,.145 if tid=='L3-03' else 0];eye=target+[.02,.22,.34];focal=24
 else:eye=None
 if eye is not None:
  c.ClearXformOpOrder();c.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(0,0,1)).GetInverse());c.GetFocalLengthAttr().Set(focal)
 s.GetRootLayer().Save()
(P/'video_sources.json').write_text(json.dumps(runs,indent=2))
