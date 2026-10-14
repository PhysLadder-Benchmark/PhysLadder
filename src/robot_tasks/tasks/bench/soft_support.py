"""GPU-compatible support for the opt-in FEM experiment; source mesh unchanged."""
def configure(stage, root):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
    box=stage.GetPrimAtPath(root)
    material=UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(root+'/Colliders/LowerBody')).ComputeBoundMaterial('physics')[0]
    if not material:raise ValueError('Missing native support contact material')
    disabled=[]
    for prim in Usd.PrimRange(box):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False);disabled.append(str(prim.GetPath()))
    # Coordinates in the ORIGINAL 190 x 200 x 100 mm authoring frame. Parent
    # scale supplies the confirmed 200 mm width. Five low-vertex convex meshes
    # avoid GPU cooking failures on tiny decorative fillets.
    parts=[('Floor',(0,0,.0935/2),(.190,.200,.0935)),
           ('Left',(-.091,0,.05),(.008,.200,.100)),
           ('Right',(.091,0,.05),(.008,.200,.100)),
           ('Front',(0,-.096,.05),(.174,.008,.100)),
           ('Back',(0,.096,.05),(.174,.008,.100))]
    paths=[]
    for name,center,size in parts:
        cube=UsdGeom.Cube.Define(stage,root+'/FEMContact/'+name)
        cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateVisibilityAttr('invisible');prim=cube.GetPrim();UsdPhysics.CollisionAPI.Apply(prim)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose='physics')
        c=PhysxSchema.PhysxCollisionAPI.Apply(prim);c.CreateContactOffsetAttr(.0002);c.CreateRestOffsetAttr(0.)
        paths.append(str(prim.GetPath()))
    return dict(revision='fem_support_five_primitives_v1',enabled_colliders=paths,
                disabled_native_colliders=len(disabled),source_asset_modified=False,
                limitation='Decorative grooves and rounded perimeter collision simplified; outer dimensions, recess floor and rim heights preserved')
