"""Pinned assets and explicit CPU collision adaptation; no policy access."""

import hashlib
import json

from ...paths import PROJECT_ROOT

ROOT = PROJECT_ROOT / "assets/l1"

# L1-07 photo variant: retain the dimensioned 190 x 200 x 100 mm box;
# match the estimated visor to the real video's near-equal box/brim width.
# This is a photo estimate, not a measured hat size.
VISOR_PHOTO_SCALE = 0.8

MARBLE_PROPS = tuple('marble_' + color + '_20261011' for color in
                     ('yellow_green', 'red_white', 'aqua_blue', 'emerald_green', 'clear_mint'))


def resize_visor_for_photo(stage, parent):
    """Bake the same uniform scale into visuals, colliders and reference frames.

    The rigid-body root stays unscaled. Keep the 65-g mass estimate and scale
    centre of mass by s and inertia by s². Source layers remain immutable.
    """
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(parent)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float) * VISOR_PHOTO_SCALE
            mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in points])
            mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])
        if prim != root and UsdGeom.Xformable(prim):
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(op.Get() * VISOR_PHOTO_SCALE)
                else:
                    raise ValueError("Unexpected visor transform in photo adapter: " + str(prim.GetPath()))
    mass = UsdPhysics.MassAPI(root)
    mass.GetCenterOfMassAttr().Set(mass.GetCenterOfMassAttr().Get() * VISOR_PHOTO_SCALE)
    mass.GetDiagonalInertiaAttr().Set(mass.GetDiagonalInertiaAttr().Get() * VISOR_PHOTO_SCALE**2)


def verified_asset(name):
    lock = json.loads((ROOT / "source.lock.json").read_text(encoding="utf-8"))
    item = next((x for x in lock["files"] if x["path"] == name), None)
    if item is None:
        raise ValueError(f"Unregistered L1 asset: {name}")
    for filename in ("metadata.json", "object.usdz"):
        path = ROOT / "source" / name / filename
        if not path.is_file():
            raise FileNotFoundError("L1 assets missing: run python3 tools/fetch_l1_assets.py")
        if hashlib.sha256(path.read_bytes()).hexdigest() != item[filename]["sha256"]:
            raise ValueError(f"L1 asset checksum mismatch: {name}/{filename}")
    return ROOT / "source" / name / "object.usdz", {
        "repository": lock["repository"],
        "revision": lock["revision"],
        **item,
    }


def import_asset(stage, parent, name, size, rotation, *, dynamic, task_id=None):
    """Keep source meshes/materials, normalize their bounds, unify rigid body.

    Source SDF shapes are replaced by convex decomposition for free bodies and
    triangle meshes for fixed fixtures because the shared executor uses CPU
    PhysX. This declared variant is common to Linux and Windows.
    """
    if name.startswith("tuojing:"):
        if size or any(rotation):
            raise ValueError("Tuojing props retain native metre dimensions and axes; scale/rotation must not be hidden in import")
        return import_tuojing_asset(stage, parent, name.split(":", 1)[1],
                                    dynamic=dynamic, task_id=task_id)

    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from ...robots.transforms import origin_transform

    path, evidence = verified_asset(name)
    wrapper = UsdGeom.Xform.Define(stage, parent + "/Normalized")
    source = stage.DefinePrim(parent + "/Normalized/Source")
    source.GetReferences().AddReference(str(path))
    # Material bindings remain in the referenced subtree. Deinstance only here.
    for prim in Usd.PrimRange(source):
        if prim.IsInstance():
            prim.SetInstanceable(False)
    # Vendor extents can be stale after mesh vertices are rescaled. BBoxCache
    # trusts these authored values: the rack's extent was 100,000x too large,
    # making both its rendered shell and baked collider microscopic.
    # Override on this stage only; preserve the pinned source asset and hash.
    repaired_extents = []
    for prim in Usd.PrimRange(source):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.isfinite(points).all():
            raise ValueError(f"Invalid mesh points: {name}:{prim.GetPath()}")
        actual = np.array([points.min(axis=0), points.max(axis=0)])
        authored = mesh.GetExtentAttr().Get()
        if authored is None or not np.allclose(np.asarray(authored), actual, rtol=1e-6, atol=1e-9):
            repaired_extents.append(str(prim.GetPath().MakeRelativePath(source.GetPath())))
        mesh.CreateExtentAttr([Gf.Vec3f(*v) for v in actual])
    r = origin_transform((0, 0, 0), rotation)[:3, :3]
    bounds = (
        UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        .ComputeWorldBound(source)
        .ComputeAlignedRange()
    )
    lo, hi = np.asarray(bounds.GetMin()), np.asarray(bounds.GetMax())
    centre = (lo + hi) / 2
    extent = np.abs(r) @ (hi - lo)
    if not np.isfinite(extent).all() or min(extent) <= 0:
        raise ValueError(f"Invalid asset bounds: {name}")
    scale = np.asarray(size) / extent
    # matrix transforms column vectors; Gf uses row vectors.
    matrix = np.eye(4)
    matrix[:3, :3] = np.diag(scale) @ r
    matrix[:3, 3] = -matrix[:3, :3] @ centre
    wrapper.AddTransformOp().Set(Gf.Matrix4d(matrix.T.tolist()))
    colliders = []
    transforms = UsdGeom.XformCache()
    for prim in list(Usd.PrimRange(source)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            # Bake metre-scale collider points so PhysX does not depend on
            # the source asset's hierarchy scale conventions.
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            mesh = UsdGeom.Mesh.Define(stage, parent + f"/Collider_{len(colliders)}")
            geometry_prim = prim
            if prim.IsA(UsdGeom.Mesh):
                # Some vendor SDF inputs contain internal tetrahedral faces.
                # For a fixed triangle surface prefer its authored visual shell.
                visual = stage.GetPrimAtPath(str(prim.GetPath()).replace("/collision/", "/visual/"))
                if not dynamic and "/collision/" in str(prim.GetPath()) and visual.IsA(UsdGeom.Mesh):
                    geometry_prim = visual
                original = UsdGeom.Mesh(geometry_prim)
                points = original.GetPointsAttr().Get()
                counts = original.GetFaceVertexCountsAttr().Get()
                indices = original.GetFaceVertexIndicesAttr().Get()
            elif prim.IsA(UsdGeom.Cube):
                from itertools import product

                half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
                points = [Gf.Vec3f(*p) for p in product((-half, half), repeat=3)]
                counts = [4] * 6
                indices = [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3]
            else:
                raise ValueError(f"Unsupported authored collider: {prim.GetTypeName()}")
            matrix = transforms.GetLocalToWorldTransform(geometry_prim)
            mesh.CreatePointsAttr([Gf.Vec3f(matrix.Transform(Gf.Vec3d(*p))) for p in points])
            mesh.CreateFaceVertexCountsAttr(counts)
            mesh.CreateFaceVertexIndicesAttr(indices)
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateVisibilityAttr("invisible")
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "convexDecomposition" if dynamic else "none"
            )
            colliders.append(str(mesh.GetPath()))
            # Disabled SDF shapes can still be cooked by PhysX. Remove the old
            # collision schemas entirely from the stage override, not the asset.
            for schema in list(prim.GetAppliedSchemas()):
                if "CollisionAPI" in schema or "MeshCollisionAPI" in schema:
                    prim.RemoveAPI(schema)
    if not colliders:
        raise ValueError(f"Asset has no authored colliders: {name}")
    return colliders, {
        **evidence,
        "normalized_size_m": list(size),
        "canonical_rotation_rpy": list(rotation),
        "bounds_source": "mesh extents recomputed from actual vertices",
        "repaired_mesh_extents": repaired_extents,
        "collision_adapter": "convexDecomposition" if dynamic else "static triangle mesh",
    }


def shallow_storage_pocket(stage, parent):
    """L1-03 photo variant: raise the shoulder seat 25 mm, keep the outer shell.

    Update visual and collision vertices together. The 22-mm blade hole stays
    open. This is a session-layer override, never an edit to the locked USD.
    """
    import numpy as np
    from pxr import Gf, Usd, UsdGeom

    wall_scale = (.055 - .045) / (.055 - .020)
    changed = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(parent)):
        name = prim.GetName()
        if not prim.IsA(UsdGeom.Mesh) or not name.startswith(("PocketFloor", "PocketWall")):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        if name.startswith("PocketFloor"):
            points[:,2] += .025  # Retain the authored 0.8-mm floor thickness.
        else:
            points[:,2] = .045 + (points[:,2] - .020) * wall_scale
            normals = mesh.GetNormalsAttr().Get()
            if normals is not None and len(normals):
                normals = np.asarray(normals, dtype=float)
                normals[:,2] /= wall_scale
                normals /= np.maximum(np.linalg.norm(normals, axis=1)[:,None], 1e-12)
                mesh.GetNormalsAttr().Set([Gf.Vec3f(*n) for n in normals])
        mesh.GetPointsAttr().Set([Gf.Vec3f(*p) for p in points])
        mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])
        changed.append(str(prim.GetPath()))
    if not changed:
        raise ValueError("Storage holder has no native pocket geometry")


def import_tuojing_asset(stage, parent, name, *, dynamic, task_id=None):
    """Reference native geometry, material and origin; apply declared task overrides."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from ...asset_delivery import verified_prop

    if name not in {"whiteboard_marker", "ripple_paper_cup", "black_ceramic_mug", "usb_charger", "plug_storage_base", "power_strip", "wooden_rack", "artificial_rose", "yellow_visor",
                    "capped_marker_20261011", "black_ceramic_mug_20261011", "usb_charger_20261010",
                    "power_strip_20261010", "wooden_rack_20261011", "yellow_visor_20261011",
                    "green_support_box_20261011", "ripple_paper_cup_20261011", "cream_tulip_20261011", *MARBLE_PROPS}:
        raise ValueError(f"Tuojing prop has no validated task adapter yet: {name}")
    path, evidence = verified_prop(name, task_id=task_id)
    prim = stage.GetPrimAtPath(parent)
    prim.GetReferences().AddReference(str(path))
    if not dynamic:
        # A fixed receptacle has static colliders, not a driven/teleported body.
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        prim.RemoveAppliedSchema("PhysxRigidBodyAPI")
    overrides = []
    if name == "yellow_visor_20261011":
        resize_visor_for_photo(stage, parent)
        overrides.append("L1-07 photo variant: visor uniformly scaled to 80% (about 188 x 238 x 69 mm); visuals, colliders, reference frames, COM and inertia agree; estimated 65-g mass retained; dimensioned 190 x 200 x 100 mm box unchanged")
    if name == "plug_storage_base":
        shallow_storage_pocket(stage, parent)
        overrides.append("L1-03 photo variant: pocket floor 20 -> 45 mm, recess 35 -> 10 mm; visuals/colliders updated together; outer 55-mm height and 22-mm through-hole retained")
    if name == "capped_marker_20261011":
        # L1 transfers a CLOSED marker, not a cap-removal assembly. Preserve all
        # authored external geometry; use a single compound rigid body. Do not
        # attach it to the hand or change immutable delivery bytes.
        for child in list(Usd.PrimRange(prim)):
            if not child:
                continue  # An internal shoe parent may already be inactive.
            if child != prim:
                for schema in (UsdPhysics.RigidBodyAPI, UsdPhysics.MassAPI):
                    child.RemoveAPI(schema)
                child.RemoveAppliedSchema("PhysxRigidBodyAPI")
            if child.IsA(UsdPhysics.Joint) or child.GetName().startswith("Shoe"):
                child.SetActive(False)
        overrides.append("L1 closed-cap variant: body and cap form one 22-g rigid compound; internal shoes/springs disabled; external native colliders unchanged")
    if name == "power_strip_20261010" and not dynamic:
        from pxr import Sdf

        housing = stage.GetPrimAtPath(parent + "/Housing")
        UsdPhysics.RigidBodyAPI(housing).CreateKinematicEnabledAttr(True)
        housing.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(False)
        overrides.append("Fixed strip housing is kinematic; all 12 native contact bodies and spring joints retained with native material/contact offsets")
    if name == "whiteboard_marker":
        # The delivered capsule extends below the flat visible end and balances
        # on a rounded point. Match the barrel/end envelope with a flat cylinder
        # so an upright initial pose rests naturally, without a hidden constraint.
        barrel = UsdGeom.Cylinder.Define(stage, parent + "/Colliders/Barrel")
        barrel.CreateRadiusAttr(0.0081)
        barrel.CreateHeightAttr(0.111)
        barrel.CreateAxisAttr("Z")
        xform = UsdGeom.Xformable(barrel)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.0555))
        overrides.append("Barrel capsule replaced by radius 8.1 mm, height 111 mm flat-ended cylinder; cap capsule retained")
    colliders = [str(p.GetPath()) for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                 if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    if not colliders:
        raise ValueError(f"Tuojing prop has no active collision shapes: {name}")
    return colliders, {**evidence, "native_metres": True,
                       "origin": "delivered sphere centre" if name in MARBLE_PROPS else "delivered bottom centre",
                       "dynamic": dynamic, "source_modified": False,
                       "overrides": overrides, "collision_adapter": "native separate convex/primitives; cavity retained"}
