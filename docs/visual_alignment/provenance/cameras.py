from pxr import Usd,UsdGeom
import numpy as np,json
from pathlib import Path
R=Path('/vepfs-C区/bowenjing/Sunday/reports/unified_20261014');out={}
for p in R.glob('L*-*/scene.usda'):
 s=Usd.Stage.Open(str(p));c=UsdGeom.Camera(s.GetPrimAtPath('/World/Cameras/ThirdPerson'));out[p.parent.name]={'matrix':np.array(c.ComputeLocalToWorldTransform(0)).tolist(),'f':c.GetFocalLengthAttr().Get(),'ha':c.GetHorizontalApertureAttr().Get(),'va':c.GetVerticalApertureAttr().Get()}
Path(__file__).with_name('cameras.json').write_text(json.dumps(out))
