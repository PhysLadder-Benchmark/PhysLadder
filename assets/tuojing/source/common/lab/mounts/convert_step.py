"""Rebuild metre-scale USD geometry. pip install cadquery==2.8.0 usd-core numpy"""
from pathlib import Path
import json,numpy as np,cadquery as cq
from pxr import Usd,UsdGeom
ROOT=Path(__file__).resolve().parent
shape=cq.importers.importStep(str(ROOT/'wrist_camera_mount.step')).val()
assert shape.isValid() and len(shape.Solids())==1
vertices,faces=shape.tessellate(.05,.1)
points=np.array([v.toTuple() for v in vertices])*.001
path=ROOT/'wrist_camera_mount.usdc';path.unlink(missing_ok=True)
stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
root=UsdGeom.Xform.Define(stage,'/Mount');stage.SetDefaultPrim(root.GetPrim())
mesh=UsdGeom.Mesh.Define(stage,'/Mount/Geometry');mesh.CreatePointsAttr(points.tolist());mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(np.array(faces).ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([(.85,.86,.83)])
root.GetPrim().SetCustomDataByKey('source','wrist_camera_mount.step; millimetres converted to metres; original CAD origin preserved')
stage.GetRootLayer().Save();print(path)
