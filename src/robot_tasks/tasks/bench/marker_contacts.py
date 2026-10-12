"""Stable cap/box contacts without changing the native marker fit.

The cap's 241 small hulls and the box's overlapping rim hulls can form
contradictory contact constraints after a release. For L2-01 use five solid
box pieces and a convex cap envelope restricted to cap/box contact. The barrel,
fingers and spring shoes still interact with the original cap geometry.
"""

import numpy as np


def support_prisms(points, floor_z, top_z, half_opening):
    """Five convex prisms; preserve outer footprint, floor and rim heights.

    Only the decorative stepped grooves and inner corner fillets are omitted.
    Full-depth rim pieces have no thin lower faces that can trap another body
    against the support floor after numerical penetration.
    """
    points = sorted(set(map(tuple, np.asarray(points)[:, :2])))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def chain(items):
        result = []
        for point in items:
            while len(result) > 1 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
        return result

    outline = np.asarray(chain(points)[:-1] + chain(points[::-1])[:-1])

    def clip(polygon, axis, bound, sign):
        output = []
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            da, db = sign * (a[axis] - bound), sign * (b[axis] - bound)
            if da >= 0:
                output.append(a)
            if (da >= 0) != (db >= 0):
                output.append(a + (b - a) * da / (da - db))
        return np.asarray(output)

    hx, hy = half_opening
    centre_strip = clip(clip(outline, 0, -hx, 1), 0, hx, -1)
    return [("Floor", outline, floor_z),
            ("Left", clip(outline, 0, -hx, -1), top_z),
            ("Right", clip(outline, 0, hx, 1), top_z),
            ("Front", clip(centre_strip, 1, -hy, -1), top_z),
            ("Back", clip(centre_strip, 1, hy, 1), top_z)]


def configure_marker_box_contacts(stage, marker_path, box_path):
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    box = stage.GetPrimAtPath(box_path)
    shell = stage.GetPrimAtPath(box_path + "/Visuals/SteppedMoldedShell")
    if not shell or not shell.IsA(UsdGeom.Mesh):
        raise ValueError("Marker support box requires its delivered closed shell")
    if not UsdPhysics.RigidBodyAPI(box).GetKinematicEnabledAttr().Get():
        raise ValueError("Marker support must be fixed/kinematic")

    cap = stage.GetPrimAtPath(marker_path + "/Cap")
    native = [p for p in Usd.PrimRange(cap)
              if p.HasAPI(UsdPhysics.CollisionAPI)
              and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    if not native or any(not p.IsA(UsdGeom.Mesh) for p in native):
        raise ValueError("Unexpected native marker-cap collision geometry")
    transforms = UsdGeom.XformCache()
    cap_inverse = np.linalg.inv(np.asarray(transforms.GetLocalToWorldTransform(cap)).T)
    vertices = []
    for shape in native:
        matrix = cap_inverse @ np.asarray(transforms.GetLocalToWorldTransform(shape)).T
        points = np.asarray(UsdGeom.Mesh(shape).GetPointsAttr().Get())
        vertices.append(points @ matrix[:3, :3].T + matrix[:3, 3])
    vertices = np.concatenate(vertices)
    radius = float(np.linalg.norm(vertices[:, :2], axis=1).max())
    lower, upper = map(float, (vertices[:, 2].min(), vertices[:, 2].max()))
    if not 0 < radius < 0.02 or not 0 < upper - lower < 0.10:
        raise ValueError("Unexpected marker-cap size; audit the new asset first")

    # Preserve each delivered contact material, including friction/restitution.
    box_material = UsdShade.MaterialBindingAPI(
        stage.GetPrimAtPath(box_path + "/Colliders/LowerBody")
    ).ComputeBoundMaterial("physics")[0]
    cap_material = UsdShade.MaterialBindingAPI(native[0]).ComputeBoundMaterial("physics")[0]
    if not box_material or not cap_material:
        raise ValueError("Missing native marker/box physics material")
    for shape in Usd.PrimRange(box):
        if shape.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(shape).CreateCollisionEnabledAttr(False)
    shell_points = np.asarray(UsdGeom.Mesh(shell).GetPointsAttr().Get())
    inset = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(
        box_path + "/Colliders/InsetFace")).GetPointsAttr().Get())
    floor_z, top_z = float(inset[:, 2].max()), float(shell_points[:, 2].max())
    half_opening = np.abs(inset[:, :2]).max(axis=0)
    box_shapes = []
    for name, outline, height in support_prisms(shell_points, floor_z, top_z, half_opening):
        n = len(outline)
        points = [(float(x), float(y), z) for z in (0., height) for x, y in outline]
        faces = []
        for i in range(1, n - 1):
            faces.extend([(0, i + 1, i), (n, n + i, n + i + 1)])
        for i in range(n):
            j = (i + 1) % n
            faces.extend([(i, j, n + j), (i, n + j, n + i)])
        mesh = UsdGeom.Mesh.Define(stage, box_path + "/ContactSolids/" + name)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        mesh.CreateVisibilityAttr("invisible")
        shape = mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(shape)
        UsdPhysics.MeshCollisionAPI.Apply(shape).CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(shape).Bind(box_material, materialPurpose="physics")
        box_shapes.append(shape)

    envelope = UsdGeom.Cylinder.Define(stage, str(cap.GetPath()) + "/BoxContactEnvelope")
    envelope.CreateAxisAttr("Z")
    envelope.CreateRadiusAttr(radius)
    envelope.CreateHeightAttr(upper - lower)
    envelope.AddTranslateOp().Set((0, 0, (lower + upper) / 2))
    envelope.CreateVisibilityAttr("invisible")
    proxy = envelope.GetPrim()
    UsdPhysics.CollisionAPI.Apply(proxy)
    UsdShade.MaterialBindingAPI.Apply(proxy).Bind(cap_material, materialPurpose="physics")
    for shape in (*box_shapes, proxy):
        contact = PhysxSchema.PhysxCollisionAPI.Apply(shape)
        contact.CreateContactOffsetAttr(0.0002)
        contact.CreateRestOffsetAttr(0.0)

    # Only the cap/box pair uses the envelope. In particular, it must NOT fill
    # the cap cavity against the barrel or change what the robot can grasp.
    shoe_shapes = [p for i in range(4)
                   for p in Usd.PrimRange(stage.GetPrimAtPath(f"{marker_path}/Shoe{i}"))
                   if p.HasAPI(UsdPhysics.CollisionAPI)]
    others = [p for p in stage.Traverse()
              if p.HasAPI(UsdPhysics.CollisionAPI)
              and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
              and p != proxy and p not in box_shapes + native + shoe_shapes]
    group_root = marker_path + "/BoxContactGroups"

    def group(name, shapes):
        result = UsdPhysics.CollisionGroup.Define(stage, group_root + "/" + name)
        result.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
            [p.GetPath() for p in shapes]
        )
        return result

    support = group("Support", box_shapes)
    fit = group("NativeCapAndShoes", native + shoe_shapes)
    exterior = group("CapEnvelope", [proxy])
    rest = group("OtherColliders", others)
    fit.CreateFilteredGroupsRel().SetTargets([support.GetPath()])
    exterior.CreateFilteredGroupsRel().SetTargets([rest.GetPath(), fit.GetPath()])
    return {
        "revision": "marker-box-contact-v1",
        "box_collision": "five full-depth convex solids, preserving delivered outer footprint, rim/floor heights and recess width",
        "box_floor_z_m": floor_z,
        "box_rim_z_m": top_z,
        "box_recess_half_extents_m": half_opening.tolist(),
        "cap_box_collision": "convex cylinder envelope; cap/box pair only",
        "cap_radius_m": radius,
        "cap_axial_bounds_m": [lower, upper],
        "native_cap_shapes_retained": len(native),
        "native_fit_and_finger_contacts_preserved": True,
        "native_mass_inertia_materials_and_springs_preserved": True,
        "source_assets_modified": False,
        "limitation": "Box decorative grooves/inner corner fillets are simplified. Box edges cannot enter the cap cavity/top recess; native barrel-cap fit is unchanged.",
    }
