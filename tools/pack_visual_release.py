"""Build a relocatable snapshot; requires pxr, does not run dynamics."""
from pathlib import Path
from pxr import Usd,UsdUtils,Sdf
import argparse,hashlib,json,os,shutil
ap=argparse.ArgumentParser();ap.add_argument('--sunday-root',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
root=a.sunday_root;src=root/'reports/visual_alignment_r3_20261014';out=a.output
out.mkdir(parents=True,exist_ok=False)
BUILTINS={'OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl'}
manifest={'version':'visual_alignment_r3_20261014','physics_rerun':False,'runtime':'Isaac Sim 5.1 with NVIDIA built-in MDL materials','builtin_materials':sorted(BUILTINS),'tasks':{},'dependencies':{}}
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def export(layer,dest):
 dest.parent.mkdir(parents=True,exist_ok=True)
 def remap(x):
  if not x or x in BUILTINS:return x
  p=Path(x)
  if not p.is_file():raise FileNotFoundError(x)
  digest=sha(p);rel=Path('textures')/(digest[:20]+p.suffix.lower());target=out/rel
  if not target.exists():target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  manifest['dependencies'][x]={'path':str(rel),'sha256':digest}
  return os.path.relpath(target,dest.parent)
 UsdUtils.ModifyAssetPaths(layer,remap);layer.Export(str(dest))
def split(layer,dest,keep):
 copy=Sdf.Layer.CreateAnonymous();copy.TransferContent(layer);st=Usd.Stage.Open(copy)
 for pr in list(st.GetPrimAtPath('/World').GetChildren()):
  if pr.GetName() not in keep:st.RemovePrim(pr.GetPath())
 for pr in list(st.GetPseudoRoot().GetChildren()):
  if pr.GetName()!='World':st.RemovePrim(pr.GetPath())
 st.SetDefaultPrim(st.GetPrimAtPath('/World'));export(copy,dest)
env={'Materials','Room','Furniture','Lights','Cameras','DetailCameras','ReferenceDetail','VisualRevision'}
nonobjects=env|{'Robots','PhysicsScene','GroundPlane'}
for td in sorted(src.glob('L*')):
 if not td.is_dir():continue
 tid=td.name;entry={}
 for mode in ['static','video']:
  original=td/mode/'scene.usdc';st=Usd.Stage.Open(str(original));layer=st.Flatten()
  # All root asset references must resolve before rewriting.
  world=st.GetPrimAtPath('/World');names={p.GetName() for p in world.GetChildren()}
  objects=names-nonobjects
  if mode=='static':
   split(layer,out/tid/'environment.usdc',env)
   split(layer,out/tid/'task_objects.usdc',objects|{'Materials'})
   entry['environment']=f'{tid}/environment.usdc';entry['task_objects']=f'{tid}/task_objects.usdc';entry['object_roots']=['/World/'+n for n in sorted(objects)]
  dest=out/tid/mode/'scene.usdc';export(layer,dest)
  entry[mode]=str(dest.relative_to(out))
  entry[mode+'_source_sha256']=sha(original)
  if mode=='video':
   for f in ['motion.npz','soft.npz','physics.npz','tactile.npz','taxel_projection.npz','thread_motion.npz','recording.json','frame_times.json']:
    p=td/mode/f
    if p.exists():shutil.copy2(p,dest.parent/f)
 manifest['tasks'][tid]=entry;print('PACKED',tid,flush=True)
# L3-11 is an explicitly preserved baseline; no r3 adjustments or replay supplied.
p=root/'reports/unified_20261014/L3-11/scene.usda';export(Usd.Stage.Open(str(p)).Flatten(),out/'L3-11/static/scene.usdc')
manifest['tasks']['L3-11']={'static':'L3-11/static/scene.usdc','status':'unchanged baseline, excluded from r3','source_sha256':sha(p)}
for name in ['shared_look.json','table_map.json','修订记录.json','validation.json','video_sources.json','交付检查.json','发布记录_r3.json']:
 shutil.copy2(src/name,out/name)
shutil.copy2(src/'render_revision.py',out/'render_revision.py')
(out/'scene_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
files={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()}
(out/'SHA256SUMS.json').write_text(json.dumps(files,ensure_ascii=False,indent=2))
print('DONE',len(files),sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),flush=True)
