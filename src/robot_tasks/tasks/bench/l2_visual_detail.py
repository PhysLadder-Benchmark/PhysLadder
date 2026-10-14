"""Photo-measured appearance overrides. No camera, task pose or dynamics execution."""
import json
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade, UsdPhysics, Vt
from ...paths import PROJECT_ROOT

ASSETS=PROJECT_ROOT/'assets/real2sim_l2_20261013/reference_detail'

def apply(stage,task_id):
    data=json.loads((ASSETS/'projection.json').read_text())
    C=np.asarray(data['camera_to_world']);world_to_camera=np.linalg.inv(C)
    original=stage.GetPrimAtPath('/World/Furniture/Worktable/Top')
    bb=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(original).ComputeAlignedRange()
    lo,hi=np.array(bb.GetMin()),np.array(bb.GetMax())
    nx,ny=768,512
    xx,yy=np.meshgrid(np.linspace(lo[0],hi[0],nx+1),np.linspace(lo[1],hi[1],ny+1))
    vertices=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,hi[2]+.0001)]
    cells=(np.arange(ny)[:,None]*(nx+1)+np.arange(nx)[None,:]).ravel()
    faces=np.concatenate([np.c_[cells,cells+1,cells+nx+2],np.c_[cells,cells+nx+2,cells+nx+1]])
    texture_vertices=vertices.copy()
    # Keep existing grain attached to the table when its physical height changes.
    texture_vertices[:,2]-=float(stage.GetPrimAtPath("/World/Furniture/Worktable").GetCustomDataByKey("table_lift_m") or 0.)
    q=texture_vertices@world_to_camera[:3,:3].T+world_to_camera[:3,3]
    u=.5+data['focal']/data['ha']*q[:,0]/-q[:,2]
    v=.5-data['focal']/data['va']*q[:,1]/-q[:,2]
    # Sample the unmodified source photograph onto a dense world-fixed surface.
    # Vertex colours avoid hard UV seams and do not modify or retouch the photo.
    from PIL import Image
    source=PROJECT_ROOT/'assets/real2sim_l2_20261013/reference_appearance/l207_initial_reference.png'
    rgb=np.asarray(Image.open(source).convert('RGB'),dtype=float)/255.
    rgb=np.where(rgb<=.04045,rgb/12.92,((rgb+.055)/1.055)**2.4)
    def sample(a,b):
        x=np.clip(a,0,1)*(rgb.shape[1]-1);y=np.clip(b,0,1)*(rgb.shape[0]-2);x0=np.floor(x).astype(int);y0=np.floor(y).astype(int);x1=np.minimum(x0+1,rgb.shape[1]-1);y1=y0+1;wx=(x-x0)[:,None];wy=(y-y0)[:,None]
        return (1-wy)*((1-wx)*rgb[y0,x0]+wx*rgb[y0,x1])+wy*((1-wx)*rgb[y1,x0]+wx*rgb[y1,x1])
    # Occluder silhouette measured from the source frame, with a margin to avoid
    # putting photographed drawers, cables or robot parts onto the virtual table.
    limit=np.interp(v,[.50,.52,.54,.57,.61,.68,.78,.88,1.],[1.,.89,.795,.785,.775,.765,.738,.707,.697])
    smooth=lambda z: np.clip(z,0,1)**2*(3-2*np.clip(z,0,1))
    weight=smooth((v-.505)/.055)*smooth((limit-u)/.038)
    # Same-photo extension for unseen areas: reflect into clean nearby table,
    # then fade across the boundary. This does not reconstruct hidden grain.
    fu=np.where(u>limit-.019,2*(limit-.019)-u,u)
    fv=np.where(v<.535,1.07-v,v)
    fu=np.clip(fu,.05,.67);fv=np.clip(fv,.55,.985)
    colors=np.array([0.708026, 0.645367, 0.565408])*(weight[:,None]*sample(u,v)+(1-weight[:,None])*sample(fu,fv))
    mesh=UsdGeom.Mesh.Define(stage,'/World/ReferenceDetail/TableSurface')
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)));mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(False)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('displayColor',Sdf.ValueTypeNames.Color3fArray,'vertex').Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
    mat=UsdShade.Material.Define(stage,'/World/Materials/ReferenceTableGrain')
    surf=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');surf.CreateIdAttr('UsdPreviewSurface');surf.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.84);surf.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.);surf.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.08));surf.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
    reader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Colour');reader.CreateIdAttr('UsdPrimvarReader_float3');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor');reader.CreateOutput('result',Sdf.ValueTypeNames.Float3)
    surf.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(),'result');surf.CreateOutput('surface',Sdf.ValueTypeNames.Token);mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    result={'revision':'measured_apertures_projected_grain_v2','table_texture':'Unmodified real L2-07 image sampled on a dense world-fixed vertex-colour surface; diffuse material, not a camera overlay','observed_vertices':int((weight>.999).sum()),'estimated_or_blended_vertices':int((weight<=.999).sum()),'caveat':'Captured illumination is present in source; occluded/unseen table grain uses a same-photo extension, not recovered ground truth. All cameras unchanged.'}
    if task_id=='L2-07':
        root='/World/L2/organizer/Body/Visuals'
        old=UsdGeom.Mesh(stage.GetPrimAtPath(root+'/PerforatedBase'));UsdGeom.Imageable(old).MakeInvisible()
        new=UsdGeom.Mesh.Define(stage,root+'/ReferencePerforatedBase');new.GetPrim().GetReferences().AddReference(str(ASSETS/'perforated_base.usda'),'/Base');material=UsdShade.MaterialBindingAPI(old).ComputeBoundMaterial()[0];UsdShade.MaterialBindingAPI.Apply(new.GetPrim()).Bind(material)
        collider_root='/World/L2/organizer/Body/Colliders'
        old_colliders=[p for p in stage.Traverse() if str(p.GetPath()).startswith(collider_root+'/PerforatedBase_')]
        physical_material=UsdShade.MaterialBindingAPI(old_colliders[0]).ComputeBoundMaterial(materialPurpose='physics')[0]
        for prim in old_colliders:UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        new_colliders=UsdGeom.Xform.Define(stage,collider_root+'/ReferenceBase');new_colliders.GetPrim().GetReferences().AddReference(str(ASSETS/'perforated_collision.usda'),'/Colliders')
        if physical_material:UsdShade.MaterialBindingAPI.Apply(new_colliders.GetPrim()).Bind(physical_material,materialPurpose='physics')
        # The same dimensional change is applied to visible partitions AND their
        # matching fixed collision meshes. Articulation, masses and pose stay put.
        for group in [root,collider_root]:
            for name,new_y in [('LongDivider0',-.009),('LongDivider1',.054)]:
                m=UsdGeom.Mesh(stage.GetPrimAtPath(group+'/'+name));points=np.array(m.GetPointsAttr().Get());points[:,1]+=new_y-(points[:,1].min()+points[:,1].max())/2;m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)));m.GetExtentAttr().Set([Gf.Vec3f(*points.min(0).tolist()),Gf.Vec3f(*points.max(0).tolist())])
            for name in ['FrontCellDivider1','FrontCellDivider2']:
                m=UsdGeom.Mesh(stage.GetPrimAtPath(group+'/'+name));points=np.array(m.GetPointsAttr().Get());y0,y1=points[:,1].min(),points[:,1].max();points[:,1]=y0+(points[:,1]-y0)*(-.009-y0)/(y1-y0);points[:,2]*=(.010 if name=="FrontCellDivider2" else .016)/points[:,2].max();m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)));m.GetExtentAttr().Set([Gf.Vec3f(*points.min(0).tolist()),Gf.Vec3f(*points.max(0).tolist())])
            m=UsdGeom.Mesh(stage.GetPrimAtPath(group+'/RightSide'));points=np.array(m.GetPointsAttr().Get());points[:,2]*=.031/points[:,2].max();m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)));m.GetExtentAttr().Set([Gf.Vec3f(*points.min(0).tolist()),Gf.Vec3f(*points.max(0).tolist())])
        result['perforations']={'measured_visible_apertures':41,'true_visual_mesh_openings':True,'collision':'Fixed aperture and divider meshes follow the visual geometry; masses/joints retained. Static verified only, no contact calibration.'}
    return result
