"""Verify hashes and USD reference closure; no simulation is run."""
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--usd',action='store_true');a=p.parse_args();root=a.root.resolve()
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
expected=json.loads((root/'SHA256SUMS.json').read_text());errors=[]
for name,want in expected.items():
 f=root/name
 if not f.is_file():errors.append('missing '+name)
 elif sha(f)!=want:errors.append('hash '+name)
stages=0
if a.usd:
 from pxr import Usd,Sdf,UsdUtils
 allowed=set(json.loads((root/'scene_manifest.json').read_text())['builtin_materials'])
 for f in sorted(root.rglob('*.usdc')):
  layer=Sdf.Layer.FindOrOpen(str(f));st=Usd.Stage.Open(layer)
  if not st:errors.append('USD open '+str(f));continue
  stages+=1
  def inspect(x):
   if not x or x in allowed:return x
   t=(f.parent/x).resolve()
   if Path(x).is_absolute() or not t.is_relative_to(root) or not t.is_file():errors.append(f'{f.relative_to(root)}: invalid dependency {x}')
   return x
  # Scan a copy; verification never saves or changes source layers.
  copy=Sdf.Layer.CreateAnonymous();copy.TransferContent(layer);UsdUtils.ModifyAssetPaths(copy,inspect)
  if f.name in ('environment.usdc','task_objects.usdc'):
   if st.GetPrimAtPath('/World/Robots'):errors.append('unexpected robots '+str(f))
   if not st.GetDefaultPrim():errors.append('missing default prim '+str(f))
print(json.dumps({'files':len(expected),'usd_stages':stages,'errors':errors,'physics_run':False},ensure_ascii=False,indent=2));raise SystemExit(bool(errors))
