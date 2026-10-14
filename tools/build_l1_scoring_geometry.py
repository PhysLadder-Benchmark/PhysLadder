"""Regenerate locked L1 collider metadata; requires USD Python and scipy.

Run from the repo root with an environment providing pxr and scipy. This tool
only reads the hash-locked delivery; it never edits source USDs. Mesh hulls are
separate per collider, preserving cup and visor openings. No GPT calls.
"""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from pxr import Usd,UsdGeom,UsdPhysics
result={'delivery':'2026-10-05 + 2026-10-09 + Feishu L1 2026-10-11','source':'authored enabled convex colliders, native metres; marker flat-barrel, shallow holder-pocket and 80-percent photo visor runtime overrides recorded explicitly','props':{}}
from robot_tasks.tasks.l1.assets import MARBLE_PROPS
for name in ['whiteboard_marker','ripple_paper_cup','black_ceramic_mug','usb_charger','plug_storage_base','artificial_rose','yellow_visor','wooden_rack', 'capped_marker_20261011','black_ceramic_mug_20261011','black_ceramic_mug_20261013','black_ceramic_mug_20261013_bulged_v2','black_ceramic_mug_20261013_reference_v3','usb_charger_20261010','usb_charger_measured_20261014','yellow_visor_20261011','yellow_visor_20261014_measured','green_support_box_20261014_measured','wooden_rack_20261011','wooden_rack_20261014_measured','ripple_paper_cup_20261011','ripple_paper_cup_20261013_measured','cream_tulip_20261011','green_support_box_20261011', *MARBLE_PROPS]:
 from robot_tasks.asset_delivery import verified_prop
 path,_=verified_prop(name);s=Usd.Stage.Open(str(path)); cache=UsdGeom.XformCache(); shapes=[]
 if name=='plug_storage_base':
  from robot_tasks.tasks.l1.assets import shallow_storage_pocket
  s.SetEditTarget(s.GetSessionLayer())
  shallow_storage_pocket(s,str(s.GetDefaultPrim().GetPath()))
 if name=='yellow_visor_20261011':
  from robot_tasks.tasks.l1.assets import resize_visor_for_photo
  s.SetEditTarget(s.GetSessionLayer())
  resize_visor_for_photo(s,str(s.GetDefaultPrim().GetPath()))
 for p in s.Traverse():
  if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  if name=='capped_marker_20261011' and '/Shoe' in str(p.GetPath()):continue
  if name.startswith('wooden_rack') and p.GetName()!='Hook0':continue
  d={'name':p.GetName(),'path':str(p.GetPath().MakeRelativePath(s.GetDefaultPrim().GetPath())),'kind':p.GetTypeName().lower(),'transform':np.array(cache.GetLocalToWorldTransform(p)).T.tolist()}
  if p.IsA(UsdGeom.Mesh):
   points=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(),dtype=float); hull=ConvexHull(points); chosen=sorted(hull.vertices);mapping={v:i for i,v in enumerate(chosen)}
   edges=sorted({tuple(sorted((mapping[a],mapping[b]))) for tri in hull.simplices for a,b in zip(tri,np.roll(tri,-1))})
   d.update(points=points[chosen].tolist(),edges=edges,faces=[[mapping[v] for v in tri] for tri in hull.simplices])
  elif p.IsA(UsdGeom.Sphere):
   d.update(radius=UsdGeom.Sphere(p).GetRadiusAttr().Get())
  elif p.IsA(UsdGeom.Capsule) or p.IsA(UsdGeom.Cylinder):
   shape=UsdGeom.Capsule(p) if p.IsA(UsdGeom.Capsule) else UsdGeom.Cylinder(p)
   d.update(radius=shape.GetRadiusAttr().Get(),height=shape.GetHeightAttr().Get(),axis=shape.GetAxisAttr().Get())
  else:raise ValueError(p.GetTypeName())
  if name=='whiteboard_marker' and p.GetName()=='Barrel':d.update(kind='cylinder',height=.111)
  shapes.append(d)
 result['props'][name]={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'shapes':shapes}
Path('assets/tuojing/l1_scoring_geometry.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
print({k:len(v['shapes']) for k,v in result['props'].items()})
