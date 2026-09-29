"""Pinned assets and explicit CPU collision adaptation; no policy access."""

import hashlib
import json

from ...paths import PROJECT_ROOT

ROOT = PROJECT_ROOT / "assets/l1"


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


def import_asset(stage, parent, name, size, rotation, *, dynamic):
    """Keep source meshes/materials, normalize their bounds, unify rigid body.

    Source SDF shapes are replaced by convex decomposition for free bodies and
    triangle meshes for fixed fixtures because the shared executor uses CPU
    PhysX. This declared variant is common to Linux and Windows.
    """
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
