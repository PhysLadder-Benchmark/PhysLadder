"""Preserve the visible finger recess instead of bridging it with one hull."""


def configure_finger_collisions(stage, root):
    from pxr import PhysxSchema, Usd, UsdPhysics

    records = []
    for link in ("link7", "link8"):
        parent = stage.GetPrimAtPath(root + "/" + link + "/collisions")
        if not parent:
            raise ValueError("Missing delivered R5 finger collision subtree: " + link)
        if parent.IsInstance():
            parent.SetInstanceable(False)
        shapes = [p for p in Usd.PrimRange(parent, Usd.TraverseInstanceProxies())
                  if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not shapes:
            raise ValueError("Missing delivered R5 finger collision shape: " + link)
        for prim in shapes:
            # The imported schema may live on an Xform containing the STL mesh.
            # Keep that importer-supported binding and its exact mesh transform.
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexDecomposition")
            api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            api.CreateMaxConvexHullsAttr(64)
            api.CreateHullVertexLimitAttr(64)
            api.CreateErrorPercentageAttr(0.1)
            api.CreateMinThicknessAttr(0.0001)
            api.CreateShrinkWrapAttr(True)
            api.CreateVoxelResolutionAttr(1000000)
            records.append(str(prim.GetPath()))
    return {"paths": records, "approximation": "convexDecomposition",
            "error_percentage": 0.1, "max_convex_hulls": 64,
            "shrink_wrap": True, "voxel_resolution": 1000000,
            "source": "unchanged delivered visual/STL mesh and local transform"}
