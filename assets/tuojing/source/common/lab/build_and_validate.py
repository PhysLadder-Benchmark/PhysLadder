from pathlib import Path
import os, json, math, shutil
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
os.environ.setdefault('OMNI_KIT_ALLOW_ROOT','1')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1440,'height':1080,'renderer':'RaytracedLighting','extra_args':['--/app/renderer/skipDriverVersionCheck=true']})
from pxr import Usd, UsdGeom, UsdShade, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, UsdUtils
import omni.usd, omni.replicator.core as rep
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'layout_parameters.json').read_text())
SRC=Path('/vepfs-C区/visuotactile2/asset/robots/manipulators/arx_arms/isaacsim/generated/r5a')
(ROOT/'robots').mkdir(exist_ok=True)
robot_src=Usd.Stage.Open(str(SRC/'asset.usd'))
robot_src.Flatten().Export(str(ROOT/'robots/r5a.usdc'))
shutil.copy2(SRC/'validation.json',ROOT/'robots/upstream_validation.json')
(ROOT/'tuojing_lab.usda').unlink(missing_ok=True)
stage=Usd.Stage.CreateNew(str(ROOT/'tuojing_lab.usda'))
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.0)
world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
stage.SetTimeCodesPerSecond(60)
materials={}
def mat(name,color,rough=.6,metal=0,opacity=1):
 m=UsdShade.Material.Define(stage,'/World/Materials/'+name)
 s=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Surface'));s.CreateIdAttr('UsdPreviewSurface')
 for n,t,v in [('diffuseColor',Sdf.ValueTypeNames.Color3f,Gf.Vec3f(*color)),('roughness',Sdf.ValueTypeNames.Float,rough),('metallic',Sdf.ValueTypeNames.Float,metal),('opacity',Sdf.ValueTypeNames.Float,opacity)]:s.CreateInput(n,t).Set(v)
 m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');materials[name]=m
 return m
mat('Wall',(.79,.74,.61));mat('White',(.85,.86,.83));mat('Oak',(.54,.36,.18),.55);mat('Floor',(.39,.18,.085),.8);mat('Metal',(.49,.53,.54),.28,.7);mat('Dark',(.025,.035,.038));mat('Fabric',(.33,.43,.43),.93);mat('Glass',(.64,.78,.81),.12,0,.12);mat('Felt',(.50,.48,.39),1);mat('Red',(.66,.025,.018),.36);mat('Blue',(.025,.24,.48),.42)
physical=UsdShade.Material.Define(stage,'/World/Materials/Contact')
pm=UsdPhysics.MaterialAPI.Apply(physical.GetPrim());pm.CreateStaticFrictionAttr(.7);pm.CreateDynamicFrictionAttr(.55);pm.CreateRestitutionAttr(.05)
def bind(p,m):UsdShade.MaterialBindingAPI.Apply(p).Bind(materials[m])
def collision(p):
 UsdPhysics.CollisionAPI.Apply(p)
 UsdShade.MaterialBindingAPI.Apply(p).Bind(physical,UsdShade.Tokens.weakerThanDescendants,'physics')
 phys=PhysxSchema.PhysxCollisionAPI.Apply(p);phys.CreateContactOffsetAttr(.002);phys.CreateRestOffsetAttr(0)
def group(path,pos=None):
 g=UsdGeom.Xform.Define(stage,path)
 if pos is not None:g.AddTranslateOp().Set(Gf.Vec3d(*pos))
 return g
for n in ['Room','Furniture','Robots','Props','Lights','Cameras']:group('/World/'+n)
def box(path,xyz,size,m='White',coll=True):
 b=UsdGeom.Cube.Define(stage,path);b.CreateSizeAttr(1);b.AddTranslateOp().Set(Gf.Vec3d(*xyz));b.AddScaleOp().Set(Gf.Vec3f(*size));bind(b.GetPrim(),m)
 if coll:collision(b.GetPrim())
 return b
def rounded(path,xyz,size,radius,m='White',vertical=False):
 w,d,h=size;vs=[];outline=[]
 for cx,cy,start in [(w/2-radius,d/2-radius,0),(-w/2+radius,d/2-radius,90),(-w/2+radius,-d/2+radius,180),(w/2-radius,-d/2+radius,270)]:
  for a in np.linspace(start,start+90,9):
   a=math.radians(a);outline.append((cx+radius*math.cos(a),cy+radius*math.sin(a)))
 n=len(outline)
 for z in [-h/2,h/2]:vs.extend([(x,y,z) for x,y in outline])
 counts=[n,n]+[4]*n;indices=list(reversed(range(n)))+list(range(n,2*n))
 for i in range(n):j=(i+1)%n;indices.extend([i,j,j+n,i+n])
 mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreatePointsAttr(vs);mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(indices);mesh.CreateSubdivisionSchemeAttr('none')
 mesh.AddTranslateOp().Set(Gf.Vec3d(*xyz))
 if vertical:mesh.AddRotateXOp().Set(90)
 bind(mesh.GetPrim(),m);collision(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
 return mesh
def cyl(path,xyz,r,h,m='Metal',coll=True):
 c=UsdGeom.Cylinder.Define(stage,path);c.CreateRadiusAttr(r);c.CreateHeightAttr(h);c.CreateAxisAttr('Z');c.AddTranslateOp().Set(Gf.Vec3d(*xyz));bind(c.GetPrim(),m)
 if coll:collision(c.GetPrim())
 return c
def bar(path,a,b,r,m='Metal',coll=True):
 a,b=Gf.Vec3d(*a),Gf.Vec3d(*b);c=cyl(path,(a+b)/2,r,(b-a).GetLength(),m,coll)
 q=Gf.Rotation(Gf.Vec3d(0,0,1),(b-a).GetNormalized()).GetQuat();c.AddOrientOp().Set(Gf.Quatf(q.GetReal(),Gf.Vec3f(q.GetImaginary())))
 return c
# V2: coordinate axes are registered to the source LiDAR (+X windows, +Y desk).
mat('Curtain',(.82,.80,.72),1);mat('ArmGraphite',(.035,.042,.048),.34,.55);mat('ArmMetal',(.24,.27,.29),.3,.7)
def textured(name,file):
 m=materials[name];tex=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Texture'));tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set('./textures/'+file);tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
 tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
 st=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('ST'));st.CreateIdAttr('UsdPrimvarReader_float2');st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');st.CreateOutput('result',Sdf.ValueTypeNames.Float2)
 tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
 UsdShade.Shader(stage.GetPrimAtPath(m.GetPath().AppendChild('Surface'))).GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
textured('Floor','cork_photo.jpg');textured('Oak','table_photo.jpg')
def textured_plane(path,xyz,size,m,repeat=(1,1),wall=False):
 w,h=size;vs=[(-w/2,-h/2,0),(w/2,-h/2,0),(w/2,h/2,0),(-w/2,h/2,0)]
 mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreatePointsAttr(vs);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
 mesh.AddTranslateOp().Set(Gf.Vec3d(*xyz))
 if wall:mesh.AddRotateXOp().Set(90)
 uv=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);uv.Set([(0,0),(repeat[0],0),repeat,(0,repeat[1])]);bind(mesh.GetPrim(),m)
 return mesh
box('/World/Room/Floor',(0,0,-.06),(3.04,4.13,.12),'Floor')
textured_plane('/World/Room/FloorFinish',(0,0,.001),(2.84,3.93),'Floor',(3,4))
box('/World/Room/DeskWall',(0,2.015,1.3),(3.04,.1,2.6),'Wall')
box('/World/Room/StorageWall',(0,-2.015,1.3),(3.04,.1,2.6),'Wall')
cl=P['wall_cork']['x_min'];cr=P['wall_cork']['x_max'];pcx=(cl+cr)/2
wall_parts=[(-1.42,cl,'Wall'),(cl,cr,'Floor'),(cr,1.42,'Wall')]
for i,(lo,hi,m) in enumerate(wall_parts):
 box(f'/World/Room/WallPanel_{i}',((lo+hi)/2,1.951,1.3),(hi-lo,.024,2.6),m,False)
 if m=='Floor':textured_plane('/World/Room/CorkPanel',((lo+hi)/2,1.937,1.3),(hi-lo,2.6),'Floor',(1.6,5.2),True)
trims=[-1.42,(-1.42+cl)/2,cl,cr,(cr+1.42)/2,1.42]
for i,x in enumerate(trims):box(f'/World/Room/PanelTrim{i}',(x,1.933,1.3),(.018,.012,2.6),'Metal',False)
pcx=P['desk']['center_xy_m'][0]+.01
box('/World/Room/PowerStrip',(pcx,1.912,.845),(.52,.025,.085),'White',False)
for i in range(4):
 c=cyl(f'/World/Room/Socket{i}',(pcx-.185+i*.123,1.895,.845),.025,.004,'White',False);c.AddRotateXOp().Set(90)
 for j in [-1,1]:box(f'/World/Room/SocketHole{i}_{j+1}',(pcx-.185+i*.123+j*.007,1.891,.846),(.002,.003,.006),'Dark',False)
# The scan places two tall windows on the desk-side half of the right wall.
for i,(lo,hi) in enumerate([(-1.965,-.06),(.78,1.13),(1.84,1.965)]):box(f'/World/Room/WindowPier_{i}',(1.47,(lo+hi)/2,1.3),(.1,hi-lo,2.6),'Wall')
box('/World/Room/WindowLintel',(1.47,.89,2.52),(.1,2.15,.16),'Wall');box('/World/Room/WindowSill',(1.47,.89,.07),(.1,2.15,.14),'Wall')
for i,(y,w) in enumerate([(1.485,.71),(.36,.84)]):
 p=f'/World/Room/Window_{i}';group(p);box(p+'/Glass',(1.47,y,1.29),(.01,w,2.30),'Glass')
 for j,yy in enumerate([y-w/2,y+w/2]):box(p+f'/Jamb{j}',(1.435,yy,1.29),(.07,.035,2.36))
 for j,z in enumerate([.13,.90,2.45]):box(p+f'/Rail{j}',(1.435,y,z),(.07,w,.04))
 for j,z in enumerate([.35,.60,1.2,1.55,1.9,2.23]):box(p+f'/OuterRail{j}',(1.51,y,z),(.013,w,.009),'Metal',False)
# Pleated off-white curtains, as seen in MOV; narrow gathered strips leave the windows visible.
for k,y in enumerate([-.13,1.02,1.92]):
 p=f'/World/Room/Curtain{k}';group(p)
 for j in range(13):
  yy=y+(j-6)*.019;xx=1.33+.028*math.sin(j*1.8)
  box(p+f'/Pleat{j}',(xx,yy,1.32),(.016,.024,2.45),'Curtain',False)
for i,y in enumerate([-1.965,-1.05,-.15,1.965]):box(f'/World/Room/PartitionPost{i}',(-1.455,y,1.3),(.045,.045,2.6),'Metal')
for i,(y,w) in enumerate([(.91,2.06),(-1.515,.86)]):box(f'/World/Room/PartitionGlass{i}',(-1.455,y,1.3),(.012,w,2.52),'Glass')
for i,z in enumerate([.025,2.575]):box(f'/World/Room/PartitionRail{i}',(-1.455,0,z),(.05,3.93,.045),'Metal')
door=group('/World/Room/Door',(-1.455,-1.05,0));door.AddRotateZOp().Set(75)
box('/World/Room/Door/Glass',(0,.43,1.3),(.018,.86,2.5),'Glass')
for j,y in enumerate([0,.86]):box(f'/World/Room/Door/Frame{j}',(0,y,1.3),(.04,.034,2.55),'Metal')
for j,z in enumerate([.04,2.565]):box(f'/World/Room/Door/FrameH{j}',(0,.43,z),(.04,.89,.035),'Metal')
bar('/World/Room/Door/Handle',(.055,.75,1.03),(.055,.75,1.35),.012,'Dark')
group('/World/Room/Ceiling');box('/World/Room/Ceiling/Slab',(0,0,2.66),(3.04,4.13,.12),'Felt')
for i,y in enumerate([-1.3,1.3]):box(f'/World/Room/Ceiling/Beam{i}',(0,y,2.5),(2.84,.18,.2),'Oak')
for i,x in enumerate([-1.38,1.38]):box(f'/World/Room/Skirting{i}',(x,0,.06),(.025,3.93,.12),'Oak')
# Current video layout: tabletop LONG EDGE runs along the desk wall.
H=P['desk']['size_m'][2];desk_center=P['desk']['center_xy_m'];L,D,_=P['desk']['size_m'];th=P['desk']['top_thickness_m'];t='/World/Furniture/Worktable';group(t,(*desk_center,0))
topmesh=rounded(t+'/Top',(0,0,H-th/2),(L,D,th),P['desk']['corner_radius_m'],'Oak')
points=topmesh.GetPointsAttr().Get();uv=UsdGeom.PrimvarsAPI(topmesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);uv.Set([(float(v[0]/L+.5),float(v[1]/D+.5)) for v in points])
for i,x in enumerate([-.56,.56]):
 rounded(t+f'/Foot{i}',(x,0,.025),(.095,.69,.05),.023)
 box(t+f'/LegOuter{i}',(x,0,.265),(.075,.10,.46))
 box(t+f'/LegInner{i}',(x,0,.59),(.060,.080,.32))
 box(t+f'/TopSupport{i}',(x,0,H-.04),(.08,.64,.03))
box(t+'/Crossbar',(0,0,H-.07),(1.19,.07,.06));rounded(t+'/Control',(-.52,-D/2+.004,H-.055),(.22,.06,.025),.018,'Dark')
# Camera and appliance remain as important spatial landmarks; small loose clutter is omitted.
ap='/World/Furniture/Appliance';group(ap,(desk_center[0]+L/2+.22,desk_center[1]+.16,0))
rounded(ap+'/Body',(0,0,.46),(.38,.40,.90),.035,'White')
for i in range(9):box(ap+f'/Vent{i}',(0,-.202,.76+i*.013),(.32,.003,.004),'Dark',False)
box(ap+'/LaptopBase',(0,0,.924),(.34,.24,.018),'Metal',False)
g=group(ap+'/LaptopScreen',(0,.10,.93));g.AddRotateXOp().Set(-15);rounded(ap+'/LaptopScreen/Body',(0,0,.115),(.34,.23,.013),.008,'Dark',True)
# Storage is fixed: broad open shelves beside the entrance, closed cabinet toward window wall.
s='/World/Furniture/OpenShelf';group(s,(-.30,-1.73,.58))
box(s+'/Back',(0,-.19,.67),(1.12,.025,1.34))
for i,x in enumerate([-.56,.56]):box(s+f'/Side{i}',(x,0,.67),(.025,.4,1.34))
for i,z in enumerate([0,.335,.67,1.005,1.34]):box(s+f'/Shelf{i}',(0,0,z),(1.12,.4,.025))
c='/World/Furniture/Storage';group(c)
box(c+'/ClosedCabinet',(.72,-1.70,1.25),(.90,.48,1.34))
for i,x in enumerate([.4925,.9475]):box(c+f'/Door{i}',(x,-1.447,1.25),(.445,.026,1.32))
# Curved mesh chair back, supporting frame, armrests and five-star base.
def chair(name,x,y,angle):
 p='/World/Furniture/'+name;g=group(p,(x,y,0));g.AddRotateZOp().Set(angle)
 cyl(p+'/GasLift',(0,0,.27),.032,.35)
 rounded(p+'/Seat',(0,0,.48),(.49,.47,.07),.095,'Fabric')
 rounded(p+'/Back',(0,.22,.78),(.43,.48,.04),.10,'Fabric',True)
 rounded(p+'/Lumbar',(0,.185,.65),(.37,.13,.04),.045,'Fabric',True)
 rounded(p+'/Headrest',(0,.24,1.09),(.29,.16,.06),.055,'Fabric',True)
 # Frame bars make the ergonomic structure legible without filling the scan with clutter.
 for j,xx in enumerate([-.23,.23]):
  bar(p+f'/BackRail{j}',(xx,.235,.58),(xx,.26,.91),.013,'White')
  bar(p+f'/ArmSupport{j}',(xx*1.17,.1,.47),(xx*1.17,.1,.67),.016)
  rounded(p+f'/Armrest{j}',(xx*1.17,-.025,.69),(.07,.26,.035),.028,'Dark')
 bar(p+'/Spine',(0,.17,.42),(0,.26,1.06),.02)
 for j in range(5):
  a=j*math.tau/5;xx=.28*math.cos(a);yy=.28*math.sin(a)
  bar(p+f'/Spoke{j}',(0,0,.15),(xx,yy,.075),.018)
  w=cyl(p+f'/Wheel{j}',(xx,yy,.055),.04,.042,'Dark');w.AddRotateXOp().Set(90)
chair('Chair_A',1.04,.54,-20)
rounded('/World/Furniture/Chair_A/Footrest',(0,-.47,.41),(.38,.28,.035),.03,'Fabric')
for j,x in enumerate([-.15,.15]):bar(f'/World/Furniture/Chair_A/FootrestRail{j}',(x,-.12,.41),(x,-.48,.41),.006)
chair('Chair_B',-.90,-.85,135)
# Two identical official R5a assets, independently controlled. Facing into the room (-Y).
robot_paths=[]
for rc in P['robots']:
 name=rc['name'];x,y,z=rc['base_xyz_m'];p='/World/Robots/'+name;robot_paths.append(p)
 r=group(p,(x,y,z));r.AddRotateZOp().Set(rc['yaw_deg']);r.GetPrim().GetReferences().AddReference('./robots/r5a.usdc')
 r.GetPrim().SetCustomDataByKey('sourceScale',1.0)
 mp='/World/Furniture/'+name+'_Mount';g=group(mp,(x,y,H+.005));g.AddRotateZOp().Set(rc['yaw_deg']+90)
 box(mp+'/Plate',(0,0,0),(.21,.105,.010),'Dark')
 matname='PCB'
 if matname not in materials:mat(matname,(.025,.095,.07),.7)
 for j,xx in enumerate([-.077,.077]):
  box(mp+f'/Controller{j}',(xx,0,.011),(.045,.065,.012),matname,False)
  box(mp+f'/Connector{j}',(xx,-.027,.021),(.018,.014,.012),'White',False)
 for prim in Usd.PrimRange(r.GetPrim()):
  if prim.GetName()=='visuals':UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials['ArmGraphite'],UsdShade.Tokens.strongerThanDescendants)
  if prim.IsA(UsdPhysics.RevoluteJoint):
   d=UsdPhysics.DriveAPI.Apply(prim,'angular');d.CreateStiffnessAttr(1500);d.CreateDampingAttr(90);d.CreateMaxForceAttr(150)
  if prim.IsA(UsdPhysics.PrismaticJoint):
   d=UsdPhysics.DriveAPI.Apply(prim,'linear');d.CreateStiffnessAttr(2000);d.CreateDampingAttr(100);d.CreateMaxForceAttr(100)
 # STEP mount uses the same datum as native link6. No mirroring or nonuniform scaling.
 wp=p+'/link6/WristSensor';group(wp)
 mount=group(wp+'/Mount');mount.GetPrim().GetReferences().AddReference('./mounts/wrist_camera_mount.usdc');bind(mount.GetPrim(),'White')
 mount.GetPrim().SetCustomDataByKey('assemblyStatus',P['wrist_mount']['assembly_status'])
 # Camera housing is estimated; its back face seats on the CAD front plane.
 n=np.array(P['wrist_mount']['camera_surface_normal']);n/=np.linalg.norm(n);seat=np.array(P['wrist_mount']['camera_surface_center_m']);hp=wp+'/Sensor';hg=group(hp,(seat+n*.013).tolist());hg.AddRotateYOp().Set(math.degrees(math.atan2(-n[2],n[0])))
 box(hp+'/Housing',(0,0,0),(.026,.043,.043),'White',False)
 box(hp+'/Face',(.014,0,0),(.003,.037,.037),'Dark',False)
 for j,yy in enumerate([-.010,.010]):
  lens=cyl(hp+f'/Lens{j}',(.016,yy,0),.006,.003,'Glass',False);lens.AddRotateYOp().Set(90)
 cam=UsdGeom.Camera.Define(stage,hp+'/Camera');cam.CreateFocalLengthAttr(14.96);cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(11.787);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,10));cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(.018,0,0),Gf.Vec3d(.3,0,0),Gf.Vec3d(0,0,1)).GetInverse())
 cam.GetPrim().SetCustomDataByKey('calibrationStatus','CAD seat-constrained estimate; optical offset and intrinsics uncalibrated')
# No illustrative task clutter in default scene; add a hidden-by-default test cube only during validation.
box('/World/Props/TestCube',(desk_center[0],desk_center[1]-.18,H+.15),(.05,.05,.05),'Red');prop=stage.GetPrimAtPath('/World/Props/TestCube');UsdPhysics.RigidBodyAPI.Apply(prop);UsdPhysics.MassAPI.Apply(prop).CreateMassAttr(.08)
physics=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));physics.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateTimeStepsPerSecondAttr(60)
dome=UsdLux.DomeLight.Define(stage,'/World/Lights/Fill');dome.CreateIntensityAttr(1500);dome.CreateColorAttr(Gf.Vec3f(.85,.93,1))
for i,(x,y,z,w,h,power) in enumerate([(0,1.05,2.38,1.5,.7,8000),(0,-1.0,2.38,1.6,.8,6000)]):
 l=UsdLux.RectLight.Define(stage,f'/World/Lights/Area{i}');l.CreateIntensityAttr(power);l.CreateNormalizeAttr(True);l.CreateWidthAttr(w);l.CreateHeightAttr(h);l.CreateColorAttr(Gf.Vec3f(1,.98,.94));l.AddTranslateOp().Set(Gf.Vec3d(x,y,z))
# Long suspended linear light is a prominent video landmark.
bar('/World/Room/Pendant',(-1.14,.97,2.24),(.69,.97,2.24),.02,'White',False)
for i,x in enumerate([-1.0,.53]):bar(f'/World/Room/PendantCable{i}',(x,.97,2.25),(x,.97,2.59),.002,'Dark',False)
def camera(name,eye,target,focal=25):
 c=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);c.CreateFocalLengthAttr(focal);c.CreateClippingRangeAttr(Gf.Vec2f(.04,100));up=Gf.Vec3d(0,1,0) if np.linalg.norm(np.array(eye)[:2]-np.array(target)[:2])<1e-6 else Gf.Vec3d(0,0,1);c.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),up).GetInverse());return str(c.GetPath())
robot_mid=np.mean([r['base_xyz_m'] for r in P['robots']],axis=0)
camera('FrontAligned',(robot_mid[0],desk_center[1]-.40,1.18),(robot_mid[0],robot_mid[1],.98),14)
camera('AlignmentTop',(robot_mid[0],robot_mid[1]-.1,3),(robot_mid[0],robot_mid[1]-.1,.8),25)
align=UsdGeom.Camera(stage.GetPrimAtPath('/World/Cameras/AlignmentTop'));align.CreateProjectionAttr('orthographic');align.CreateHorizontalApertureAttr(12);align.CreateVerticalApertureAttr(8)
camera('Overview',(-4.6,5.4,5.3),(0,0,.85),27)
camera('VideoDesk',(-1.10,.15,1.65),(desk_center[0],desk_center[1],.91),21)
camera('Workcell',(-.65,.1,1.43),(desk_center[0]+.12,desk_center[1]+.15,.9),23)
camera('Storage',(-1.08,.3,1.60),(.03,-1.66,1.18),22)
camera('Windows',(-.70,-.65,1.60),(1.1,.7,1.1),22)
camera('Top',(0,-.001,7),(0,0,0),28)
top=UsdGeom.Camera(stage.GetPrimAtPath('/World/Cameras/Top'));top.CreateProjectionAttr('orthographic');top.CreateHorizontalApertureAttr(64);top.CreateVerticalApertureAttr(48)
# Camera rig: measured 1.32m mark and 0.25m setback, separately estimated optical center.
cp=P['camera'];sx,sy,_=cp['stand_xyz_m'];sp='/World/Furniture/ThirdCameraStand';group(sp)
for i,(z0,z1,rad) in enumerate([(.04,.35,.012),(.35,.62,.010),(.62,.88,.008),(.88,1.245,.006)]):
 bar(sp+f'/Tube{i}',(sx,sy,z0),(sx,sy,z1),rad,'Dark')
 cyl(sp+f'/Collar{i}',(sx,sy,z0),rad+.006,.025,'Dark')
for j,a in enumerate([90,210,330]):
 a=math.radians(a);end=(sx+.29*math.cos(a),sy+.29*math.sin(a),.014)
 bar(sp+f'/Leg{j}',(sx,sy,.16),end,.007,'Dark');bar(sp+f'/Brace{j}',(sx,sy,.065),(sx+.20*math.cos(a),sy+.20*math.sin(a),.06),.004,'Dark')
bar(sp+'/Riser',(sx,sy,1.245),(sx,sy,1.32),.009,'Dark')
# Orient housing in the same optical frame as the actual virtual sensor.
cpath=camera('ThirdPerson',cp['eye_xyz_m'],cp['target_xyz_m'],20.955/(2*math.tan(math.radians(cp['horizontal_fov_deg']/2))))
third=UsdGeom.Camera(stage.GetPrimAtPath(cpath));third.CreateHorizontalApertureAttr(20.955);third.CreateVerticalApertureAttr(20.955*cp['resolution'][1]/cp['resolution'][0]);third.GetPrim().SetCustomDataByKey('calibrationStatus',cp['intrinsics_status'])
rig=group(sp+'/Head');rig.AddTransformOp().Set(UsdGeom.Xformable(third).GetLocalTransformation())
rounded(sp+'/Head/Holder',(0,.035,.048),(.050,.185,.018),.007,'Dark');box(sp+'/Head/Clamp',(0,0,.014),(.068,.043,.015),'White',False)
# The optical center is at the front; housing stays behind the image plane.
box(sp+'/Head/Housing',(0,0,.013),(.043,.043,.026),'White',False);box(sp+'/Head/Front',(0,0,-.002),(.037,.037,.003),'Dark',False)
for j,yy in enumerate([-.01,.01]):cyl(sp+f'/Head/Lens{j}',(0,yy,-.004),.006,.002,'Glass',False)
# Calibrated handheld cameras for explicit real/render comparisons, separate from the tripod sensor.
handheld=json.loads((ROOT/'evidence/handheld_cameras.json').read_text())
for hc in handheld:
 nm='Matched_'+hc['name'].split('.')[0];c=UsdGeom.Camera.Define(stage,'/World/Cameras/'+nm);W,HH=hc['resolution'];c.CreateFocalLengthAttr(hc['K'][0][0]/W*20.955);c.CreateHorizontalApertureAttr(20.955);c.CreateVerticalApertureAttr(20.955*HH/W);c.CreateClippingRangeAttr(Gf.Vec2f(.02,50));M=np.array(hc['T_world_from_optical'])@np.diag([1,-1,-1,1]);c.AddTransformOp().Set(Gf.Matrix4d(M.T.tolist()));c.GetPrim().SetCustomDataByKey('sourceK1',hc['k1'])
# Keep the registered raw scan as optional reference, without collision or visibility by default.
reg=json.loads((ROOT/'registration.json').read_text());T=np.array(reg['source_to_world_matrix_column_vectors'])
ref=group('/World/ReferenceScan');ref.GetPrim().GetReferences().AddReference('./reference/scan.usda','/object')
A=np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1.]])
ref.AddTransformOp().Set(Gf.Matrix4d((T@A).T.tolist()));UsdGeom.Imageable(ref).MakeInvisible()
# Latest-video sparse evidence in the same metre-scale world frame, hidden by default.
ply=(ROOT/'reference/latest_video_sparse.ply').read_text().split('end_header\n',1)[1];raw=np.array([[float(v) for v in line.split()] for line in ply.splitlines() if line.strip()]);cloud=UsdGeom.Points.Define(stage,'/World/LatestVideoPoints');cloud.CreatePointsAttr(raw[:,:3].tolist());cloud.CreateWidthsAttr([.006]*len(raw));cloud.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set((raw[:,3:6]/255).tolist());UsdGeom.Imageable(cloud).MakeInvisible()
# Static scenery and robot rest targets are exported before any simulation steps.
stage.GetRootLayer().Save();print('BUILD_SAVED',flush=True)
omni.usd.get_context().open_stage(str(ROOT/'tuojing_lab.usda'))
for _ in range(25):app.update()
stage=omni.usd.get_context().get_stage()
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
sim=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
robots=[sim.scene.add(Robot(prim_path=p+'/root_joint',name=p.split('/')[-1])) for p in robot_paths];sim.reset()
qs=[np.array(rc['home_q_rad_m'],dtype=np.float32) for rc in P['robots']]
for r,q in zip(robots,qs):r.set_joint_positions(q);r.set_joints_default_state(positions=q);r.apply_action(ArticulationAction(joint_positions=q))
for _ in range(180):sim.step(render=True)
def wrist_transforms():
 c=UsdGeom.XformCache();out=[]
 for p in robot_paths:
  L=np.array(c.GetLocalToWorldTransform(stage.GetPrimAtPath(p+'/link6'))).T
  W=np.array(c.GetLocalToWorldTransform(stage.GetPrimAtPath(p+'/link6/WristSensor/Sensor/Camera'))).T
  out.append({'world':W,'relative':np.linalg.inv(L)@W})
 return out
wrist_before=wrist_transforms()
results=[]
for i,r in enumerate(robots):
 cmd=qs[i].copy();cmd[0]=(-1 if i==0 else 1)*.12;r.apply_action(ArticulationAction(joint_positions=cmd));results.append({'name':r.name,'command':cmd.tolist(),'dof_names':r.dof_names})
for _ in range(180):sim.step(render=True)
wrist_after=wrist_transforms();wrist_checks=[]
for b,a in zip(wrist_before,wrist_after):
 delta=float(np.linalg.norm(b['world'][:3,3]-a['world'][:3,3]));rigid=float(np.max(np.abs(b['relative']-a['relative'])))
 assert delta>.005 and rigid<1e-5,(delta,rigid)
 wrist_checks.append({'camera_motion_m':delta,'relative_transform_max_error':rigid,'T_link6_from_camera_usd':a['relative'].tolist()})
for i,r in enumerate(robots):
 a=r.get_joint_positions();err=float(np.max(np.abs(a-np.array(results[i]['command']))));re=float(np.max(np.abs(a[:6]-np.array(results[i]['command'])[:6])));pe=float(np.max(np.abs(a[6:]-np.array(results[i]['command'])[6:])));results[i].update(actual=a.tolist(),max_revolute_error_rad=re,max_prismatic_error_m=pe);assert np.isfinite(a).all() and re<.03 and pe<.008,results[i]
 r.apply_action(ArticulationAction(joint_positions=qs[i]))
for _ in range(120):sim.step(render=True)
xyz=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Props/TestCube')).ExtractTranslation();assert H+.02<xyz[2]<H+.035,list(xyz)
# Persist final home drives. Keep optional validation prop disabled in delivered default stage.
for path,q in zip(robot_paths,qs):
 for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
  if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in robots[0].dof_names:
   v=q[robots[0].dof_names.index(prim.GetName())];UsdPhysics.DriveAPI.Apply(prim,'angular').CreateTargetPositionAttr(float(np.degrees(v)))
  if prim.IsA(UsdPhysics.PrismaticJoint):UsdPhysics.DriveAPI.Apply(prim,'linear').CreateTargetPositionAttr(.010)
stage.GetPrimAtPath('/World/Props/TestCube').SetActive(False);stage.GetRootLayer().Save();sim.pause()
report={'wrist_rigid_attachment_checks':wrist_checks,'base_yaws_deg':[r['yaw_deg'] for r in P['robots']],'dual_arm':True,'articulations':results,'simulation_steps':480,'test_cube_final_xyz':list(xyz),'table_size_m':P['desk']['size_m'],'table_center_xy':list(desk_center),'robot_bases_xyz':[rc['base_xyz_m'] for rc in P['robots']],'axis_spacing_m':float(np.linalg.norm(np.array(P['robots'][0]['base_xyz_m'])-P['robots'][1]['base_xyz_m'])),'robot_scale':[1,1,1],'camera_configuration':cp,'collision_prims':sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse())}
# Export optical camera extrinsics relative to both robot bases.
cache=UsdGeom.XformCache();cm=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Cameras/ThirdPerson'))).T@np.diag([1,-1,-1,1]);W,HH=cp['resolution'];f=W/(2*math.tan(math.radians(cp['horizontal_fov_deg']/2)));cal={'K':[[f,0,W/2],[0,f,HH/2],[0,0,1]],'resolution':[W,HH],'T_world_from_optical':cm.tolist(),'intrinsics_status':cp['intrinsics_status'],'distortion':{'model':'none','calibrated':False},'robot_base_transforms':{}}
for rc in P['robots']:
 B=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Robots/'+rc['name']))).T;cal['robot_base_transforms'][rc['name']]={'T_world_from_base':B.tolist(),'T_base_from_optical':(np.linalg.inv(B)@cm).tolist(),'T_optical_from_base':(np.linalg.inv(cm)@B).tolist()}
(ROOT/'camera_calibration.json').write_text(json.dumps(cal,indent=2))
def capture(cam,name,cutaway=False,resolution=(1440,1080)):
 stage.SetEditTarget(stage.GetSessionLayer())
 for path in ['/World/Room/Ceiling','/World/Room/DeskWall','/World/Room/CorkPanel','/World/Room/PowerStrip']+[f'/World/Room/WallPanel_{i}' for i in range(5)]+[f'/World/Room/PanelTrim{i}' for i in range(6)]+[f'/World/Room/Socket{i}' for i in range(6)]:
  prim=stage.GetPrimAtPath(path)
  if prim.IsValid():
   im=UsdGeom.Imageable(prim);im.MakeInvisible() if cutaway else im.MakeVisible()
 rp=rep.create.render_product('/World/Cameras/'+cam,resolution);rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
 for _ in range(25):app.update()
 rep.orchestrator.step(rt_subframes=24,delta_time=0,pause_timeline=True);pixels=rgb.get_data();assert pixels.size;Image.fromarray(pixels).save(ROOT/(name+'.png'));rgb.detach(rp);rp.destroy();print('RENDER',name,flush=True)
for cam,name,cut,resol in [('AlignmentTop','alignment_top',True,(1500,1000)),('FrontAligned','front_aligned',False,(1600,900)),('Overview','overview',True,(1440,1080)),('Workcell','workcell',False,(1440,1080)),('ThirdPerson','third_person',False,(1280,720)),('Matched_IMG_3334','matched_3334',False,(736,1600)),('Matched_new_21','matched_21',False,(720,1280)),('Matched_IMG_3338','matched_3338',False,(736,1600)),('Top','top',True,(1440,1080))]:capture(cam,name,cut,resol)
_,_,missing=UsdUtils.ComputeAllDependencies(str(ROOT/'tuojing_lab.usda'));report['missing_dependencies']=[str(x) for x in missing];assert not missing,missing
(ROOT/'validation.json').write_text(json.dumps(report,indent=2));stage.SetEditTarget(stage.GetRootLayer());print('VALIDATION_PASS',json.dumps(report),flush=True);app.close()
