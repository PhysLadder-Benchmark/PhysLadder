from pathlib import Path
from pxr import Usd,UsdGeom,UsdShade,UsdLux,UsdPhysics,Sdf,Gf,Vt
import numpy as np,json,sys,hashlib
P=Path(__file__).resolve().parent;ROOT=Path('/vepfs-C区/bowenjing/Sunday');C=json.loads((P/'shared_look.json').read_text());table=json.loads((P/'table_map.json').read_text())
TASKS=['L1-01','L1-03','L1-07','L1-08','L1-09','L2-01','L2-03','L2-04','L2-06','L2-07','L3-03','L3-05','L3-14','L3-15']
def sh(s,p):return UsdShade.Shader(s.GetPrimAtPath(p))
def val(shader,k,v):
 if not shader:return
 typ=Sdf.ValueTypeNames.Bool if isinstance(v,bool) else Sdf.ValueTypeNames.Float4 if isinstance(v,(list,tuple)) and len(v)==4 else Sdf.ValueTypeNames.Color3f if isinstance(v,(list,tuple)) else Sdf.ValueTypeNames.Float
 a=shader.CreateInput(k,typ);a.DisconnectSource();a.Set(Gf.Vec4f(*v) if typ==Sdf.ValueTypeNames.Float4 else Gf.Vec3f(*v) if typ==Sdf.ValueTypeNames.Color3f else v)
def material(s,p,color,rough=.4):
 m=UsdShade.Material.Define(s,p);a=UsdShade.Shader.Define(s,p+'/Surface');a.CreateIdAttr('UsdPreviewSurface');val(a,'diffuseColor',color);val(a,'roughness',rough);a.CreateOutput('surface',Sdf.ValueTypeNames.Token);m.CreateSurfaceOutput().ConnectToSource(a.ConnectableAPI(),'surface');return m

def tube(s,p,points,r,mat):
 points=np.array(points);vs=[];fs=[]
 for i,pt in enumerate(points):
  tangent=points[min(i+1,len(points)-1)]-points[max(0,i-1)];tangent/=max(np.linalg.norm(tangent),1e-9);u=np.cross(tangent,[0,0,1]);
  if np.linalg.norm(u)<.01:u=np.cross(tangent,[0,1,0])
  u/=np.linalg.norm(u);v=np.cross(tangent,u)
  vs.extend(pt+r*(np.cos(a)*u+np.sin(a)*v) for a in np.linspace(0,2*np.pi,8,endpoint=False))
 for i in range(len(points)-1):
  for j in range(8):fs.append([i*8+j,i*8+(j+1)%8,(i+1)*8+(j+1)%8,(i+1)*8+j])
 m=UsdGeom.Mesh.Define(s,p);m.CreatePointsAttr(vs);m.CreateFaceVertexCountsAttr([4]*len(fs));m.CreateFaceVertexIndicesAttr(np.array(fs).ravel().tolist());m.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
def cube(s,p,pos,size,mat):
 c=UsdGeom.Cube.Define(s,p);c.CreateSizeAttr(1);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat);return c

def common(s,tid):
 # Only new output layers. L3-11 never enters the target list.
 for name,spec in C['lights'].items():
  pr=s.GetPrimAtPath('/World/Lights/'+name)
  if pr:
   api=UsdLux.LightAPI(pr);api.CreateIntensityAttr(spec['intensity']);api.CreateColorAttr(Gf.Vec3f(*spec['color']))
 # True surface albedo and UVs, independent of the rendering camera after authoring.
 p='/World/ReferenceDetail/TableSurface';old=s.GetPrimAtPath(p)
 if old:s.RemovePrim(p)
 lo=table['min_xy'];hi=table['max_xy'];m=UsdGeom.Mesh.Define(s,p);m.CreatePointsAttr([(lo[0],lo[1],.88015),(hi[0],lo[1],.88015),(hi[0],hi[1],.88015),(lo[0],hi[1],.88015)]);m.CreateFaceVertexCountsAttr([4]);m.CreateFaceVertexIndicesAttr([0,1,2,3]);m.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(0,0),(1,0),(1,1),(0,1)])
 mat=material(s,'/World/Materials/R3Table',[.5,.4,.3],C['table']['roughness']);a=sh(s,str(mat.GetPath())+'/Surface');uv=UsdShade.Shader.Define(s,str(mat.GetPath())+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
 tex=UsdShade.Shader.Define(s,str(mat.GetPath())+'/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(P/f'textures/table_{tid}.png')));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');val(tex,'scale',C['table']['gain']+[1]);tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3);a.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
 black=material(s,'/World/Materials/R3Cable',[.006,.008,.011],.4);white=material(s,'/World/Materials/R3Plug',[.78,.81,.79],.28)
 # Background mains plugs and cords; visual props outside manipulation space.
 for i,x in enumerate([.11455,.23755]):
  c=UsdGeom.Cylinder.Define(s,f'/World/VisualRevision/Plug{i}');c.CreateAxisAttr('Y');c.CreateRadiusAttr(.023);c.CreateHeightAttr(.026);c.AddTranslateOp().Set(Gf.Vec3d(x,1.886,.945));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(white if i==0 else black)
  tt=np.linspace(0,1,35);pts=np.c_[x+.012*np.sin(tt*2),1.87-.025*tt,.94-.23*tt];tube(s,f'/World/VisualRevision/DropCable{i}',pts,.0024,white if i==0 else black)
 tt=np.linspace(0,1,90);pts=np.c_[-.17+1.20*tt,1.54-.075*np.sin(tt*np.pi),np.full_like(tt,.885)];tube(s,'/World/VisualRevision/DesktopCable',pts,.0031,black)
 for pr in s.Traverse():
  path=str(pr.GetPath())
  if path.startswith('/World/Robots') or not pr.IsA(UsdShade.Shader):continue
  a=UsdShade.Shader(pr)
  if path.endswith('/Looks/Teal/Texture'):val(a,'scale',C['box']['texture_scale']+[1])
  if path.endswith('/Looks/Teal/Surface'):
   root=path.rsplit('/',1)[0];r=sh(s,root+'/Roughness')
   if r:
    r.GetInput('file').Set(Sdf.AssetPath(str(P/'textures/plastic_roughness.png')));r.CreateOutput('r',Sdf.ValueTypeNames.Float);a.CreateInput('roughness',Sdf.ValueTypeNames.Float).ConnectToSource(r.ConnectableAPI(),'r')
   else:val(a,'roughness',.4)
  if path.endswith('/Looks/InsetSeam/Surface'):val(a,'diffuseColor',C['box']['seam'])
  if path.endswith('/Body/Looks/Label/Surface'):val(a,'roughness',.38);val(a,'clearcoat',.12);val(a,'clearcoatRoughness',.2)
 return {'shared_config':C['revision'],'table':'UV world-rectified real wood; masked unseen areas reconstructed','background':'two mains plugs, hanging leads, desktop black lead','box':'texture tint and spatial roughness restored'}

def individual(s,tid,mode):
 changes=[]
 if tid=='L1-01':
  root='/World/Household/Fixture/Looks/'
  for n in ['MatteBlack','BlackGrain','DarkPlastic','GlossBand']:
   a=sh(s,root+n+'/Surface');val(a,'diffuseColor',[.006,.014,.045]);val(a,'roughness',.22);val(a,'clearcoat',.28);val(a,'clearcoatRoughness',.13)
  a=sh(s,root+'WhiteGlaze/Surface');val(a,'diffuseColor',[.85,.93,.98]);val(a,'roughness',.25);changes+=['mug glaze, blue-black reflections, interior and marker label gloss']
 if tid=='L1-03':
  path='/World/Household/plug_storage_base';pr=s.GetPrimAtPath(path);tf=UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(0);s.RemovePrim(path);pr=s.DefinePrim(path,'Xform');pr.GetReferences().AddReference(str(ROOT/'reports/l103_tray_measured_r2_20261014/plug_storage_base/plug_storage_base.usda'));UsdGeom.Xformable(pr).AddTransformOp().Set(tf)
  # New measured base authored in metres at table height, no inherited scale.
  for name,col in [('Ivory',[.83,.90,.91]),('Blue',[.26,.65,.70]),('CordGrey',[.58,.67,.69])]:val(sh(s,'/World/Household/Fixture/Housing/Looks/'+name+'/Surface'),'diffuseColor',col)
  changes+=['measured tray 92.5/46/24/50mm integrated','powerstrip shell and cord colour']
 if tid=='L1-07':
  for n in ['Fabric','Printed']:
   a=sh(s,f'/World/Household/hat/Looks/{n}/Texture');val(a,'scale',[.92,.94,1.0,1]);val(a,'bias',[.02,.025,.09,0])
  # Photographed front graphic replaces approximate logo artwork.
  a=sh(s,'/World/Household/hat/Looks/Printed/Texture');a.GetInput('file').Set(Sdf.AssetPath(str(P/'textures/hat_band_wrapped.png')));val(a,'scale',[.92,.94,.98,1]);val(a,'bias',[0,0,0,0]);changes+=['photographed visor graphic and fabric colour']
 if tid=='L1-09':
  for n,col in [('PetalCream',[.86,.88,.82]),('PetalInner',[.91,.92,.86]),('LeafGreen',[.035,.16,.085]),('StemGreen',[.045,.19,.105]),('VeinGreen',[.065,.23,.13])]:val(sh(s,'/World/Household/rose/Looks/'+n+'/Surface'),'diffuseColor',col)
  # Narrow the blade width about its own midline, preserve stem and contact geometry.
  for pr in s.Traverse():
   if '/rose/Visuals/Leaf' in str(pr.GetPath()) and pr.IsA(UsdGeom.Mesh):
    m=UsdGeom.Mesh(pr);v=np.array(m.GetPointsAttr().Get());centre=v.mean(0);v[:,0]=centre[0]+(v[:,0]-centre[0])*.78;m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(v.astype('f')))
  changes+=['white petals, darker green foliage, narrower leaf silhouette']
 if tid in ['L1-09','L2-04']:
  for pr in s.Traverse():
   path=str(pr.GetPath())
   if path.endswith(('/Looks/Paper/Surface','/Looks/Interior/Surface')):val(UsdShade.Shader(pr),'diffuseColor',[.84,.89,.87])
 if tid=='L2-01':changes+=['support box appearance and pen label roughness; measured dimensions retained']
 if tid=='L2-03':
  for pr in s.Traverse():
   p=str(pr.GetPath())
   if '/Looks/PaperTone' in p and p.endswith('/Surface'):
    a=UsdShade.Shader(pr);idx=int(p.split('PaperTone')[-1].split('/')[0]);val(a,'diffuseColor',[.70+.024*idx,.79+.021*idx,.81+.02*idx])
  changes+=['page-edge colour variation and reduced cyan wash']
 if tid=='L2-04':changes+=['cup colour; original inspection camera retained']
 if tid=='L2-06':
  for n in ['FrontPhoto','ReversePhoto']:
   a=sh(s,'/World/L2/orange/Looks/'+n+'/Texture');val(a,'bias',[.015,.012,.008,0]);val(a,'scale',[.94,.9,.84,1]);val(sh(s,'/World/L2/orange/Looks/'+n+'/Surface'),'roughness',.43)
  changes+=['restore orange photograph contrast and membrane detail; deformation mesh topology preserved']
 if tid=='L2-07':
  # Existing scene already carries contents; do not duplicate them.
  for pr in s.Traverse():
   if '/organizer/Contents/' in str(pr.GetPath()) and pr.IsA(UsdGeom.Imageable) and '/Collid' not in str(pr.GetPath()) and UsdGeom.Imageable(pr).ComputePurpose()!='guide':UsdGeom.Imageable(pr).MakeVisible()
  changes+=['ensure existing coloured drawer contents visible; preserve drawer joint and geometry']
 if tid=='L3-03':
  p='/World/L2/bottle_assembly/Looks/pet';mat=UsdShade.Material(s.GetPrimAtPath(p));m=UsdShade.Shader.Define(s,p+'/R3VisiblePET');m.CreateImplementationSourceAttr('sourceAsset');m.SetSourceAsset(Sdf.AssetPath('OmniSurface.mdl'),'mdl');m.SetSourceAssetSubIdentifier('OmniSurface','mdl');m.CreateOutput('out',Sdf.ValueTypeNames.Token)
  for k,v in {'enable_specular_transmission':True,'specular_transmission_weight':.86,'specular_transmission_color':[.99,.995,1.0],'diffuse_reflection_weight':.32,'diffuse_reflection_color':[.84,.88,.91],'specular_reflection_weight':1.,'specular_reflection_roughness':.11,'specular_reflection_ior':1.57,'thin_walled':False,'enable_opacity':False}.items():val(m,k,v)
  mat.CreateSurfaceOutput('mdl').ConnectToSource(m.ConnectableAPI(),'out');changes+=['unified static and video PET shader; less grey-green diffuse scattering']
 if tid=='L3-05':
  for pr in s.Traverse():
   if '/anchor_assembly/' in str(pr.GetPath()) and str(pr.GetPath()).endswith('/ZincSteel/Surface'):val(UsdShade.Shader(pr),'roughness',.21);val(UsdShade.Shader(pr),'diffuseColor',[.66,.68,.70])
  changes+=['zinc-steel specular response; threaded geometry retained']
 if tid=='L3-14':
  path='/World/L2/Fixture';pts=[]
  for i in range(4):
   pr=s.GetPrimAtPath(path+'/target_outline_'+str(i))
   if pr:
    bb=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(pr).ComputeAlignedRange();pts.extend([list(bb.GetMin()),list(bb.GetMax())]);UsdGeom.Imageable(pr).MakeInvisible()
  if pts:
   lo=np.min(pts,0);hi=np.max(pts,0);corners=np.array([[lo[0],lo[1],.8804],[hi[0],lo[1],.8804],[hi[0],hi[1],.8804],[lo[0],hi[1],.8804],[lo[0],lo[1],.8804]]);rng=np.random.default_rng(314);line=[]
   for a,b in zip(corners[:-1],corners[1:]):
    for f in np.linspace(0,1,18):q=a*(1-f)+b*f;q[:2]+=rng.normal(0,.0007,2);line.append(q)
   mat=material(s,'/World/Materials/R3RedPen',[.25,.018,.022],.72);tube(s,path+'/HandDrawnTarget',line,.00055,mat)
  val(sh(s,'/World/L2/eraser/Looks/BlueShell/Surface'),'roughness',.47);changes+=['irregular drawn target outline and less glossy eraser housing']
 if tid=='L3-15':
  val(sh(s,'/World/L2/dustpan/Looks/WarmGrey/Surface'),'diffuseColor',[.26,.29,.30]);val(sh(s,'/World/L2/dustpan/Looks/WarmGrey/Surface'),'roughness',.28)
  for i in range(5):val(sh(s,f'/World/L2/broom/Looks/Fibre{i}/Surface'),'roughness',.55)
  changes+=['neutral grey dustpan and less uniform brush highlights']
 return changes

def build(tid,source=None,mode='static'):
 assert tid!='L3-11';out=P/tid/mode;out.mkdir(parents=True,exist_ok=True);src=Path(source) if source else ROOT/'reports/unified_20261014'/tid/'scene.usda'
 s=Usd.Stage.Open(Usd.Stage.Open(str(src)).Flatten());s.RemovePrim('/Render');s.RemovePrim('/Orchestrator');rep=common(s,tid);rep['specific']=individual(s,tid,mode);rep.update(task=tid,source=str(src),physics_rerun=False,cameras_unchanged=True);s.GetRootLayer().Export(str(out/'scene.usdc'));(out/'changes.json').write_text(json.dumps(rep,indent=2));print('BUILT',tid,mode,flush=True)
if __name__=='__main__':
 for t in sys.argv[1:] or TASKS:build(t)
