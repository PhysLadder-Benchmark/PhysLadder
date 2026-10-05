"""Bind the pinned L20 laboratory delivery; never edit its source layers."""

from ..asset_delivery import verified_package


def require_lab_assets():
    """Fail before launching Kit if a private delivery is absent or modified."""
    directory, _ = verified_package("lab_v4")
    return directory / "tuojing_lab.usda"


def configure_stage(stage, camera_profile=None):
    """Only session-layer overrides; original lab coordinates remain authoritative."""
    from pxr import UsdGeom, UsdPhysics

    stage.SetEditTarget(stage.GetSessionLayer())
    if UsdGeom.GetStageUpAxis(stage) != "Z" or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        raise ValueError("The laboratory must use metres and Z-up")
    scenes = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if scenes != ["/World/PhysicsScene"]:
        raise ValueError(f"Unexpected laboratory physics scenes: {scenes}")
    camera_overrides = apply_camera_mounts(stage, camera_profile) if camera_profile else {}
    # Decorative props are not benchmark task objects. Task adapters add their
    # own physical objects and scoring without changing the source delivery.
    disabled = []
    for path in ("/World/Props", "/World/ReferenceScan", "/World/LatestVideoPoints"):
        prim = stage.GetPrimAtPath(path)
        if prim:
            prim.SetActive(False)
            disabled.append(path)
    # Include the delivered table and surrounding fixed furniture in planning.
    # Robot base mounting plates share intentional contact with the fixed base;
    # these are excluded just like the old base stands, explicitly in metadata.
    excluded = ("/World/Furniture/R5_Left_Mount/", "/World/Furniture/R5_Right_Mount/")
    colliders = tuple(str(p.GetPath()) for p in stage.Traverse()
                      if str(p.GetPath()).startswith(("/World/Furniture/", "/World/Room/"))
                      and not str(p.GetPath()).startswith(excluded)
                      and p.HasAPI(UsdPhysics.CollisionAPI)
                      and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
    return {"profile": "tuojing_lab_v4", "world_frame": "+X windows, +Y desk wall, +Z up; origin on room floor",
            "table_surface_z_m": 0.78, "arm_spacing_m": 0.440709864395438,
            "disabled_decorations": disabled, "planning_environment_paths": colliders,
            "planning_exclusions": list(excluded), "source_layers_modified": False,
            "camera_mount_overrides": camera_overrides,
            "limitations": "Estimated laboratory layout and camera calibration; not hardware calibration."}


def apply_camera_mounts(stage, profile):
    """Apply requested extrinsics to optics AND housings, only in the session layer.

    Wrist sensors remain children of link6. The overview retains its original
    heading; its configured pose also moves the tripod head and adjusts the riser.
    Lens validation in IsaacCameraRig still checks the resulting camera geometry.
    """
    import numpy as np
    from pxr import Gf, UsdGeom

    from ..robots.transforms import pose_transform, transform_pose

    stage.SetEditTarget(stage.GetSessionLayer())

    def world(path):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError(f"Missing laboratory camera component: {path}")
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T

    def set_world(path, transform):
        prim = stage.GetPrimAtPath(path)
        parent = np.array(UsdGeom.XformCache().GetParentToWorldTransform(prim)).T
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(Gf.Matrix4d((np.linalg.inv(parent) @ transform).T.tolist()))

    changes = {}
    for spec in profile.cameras:
        parent = "/World" if spec.parent == "world" else f"/World/Robots/R5_{spec.parent.title()}/link6"
        expected = "/World/Cameras/ThirdPerson" if spec.parent == "world" else parent + "/WristSensor/Sensor/Camera"
        if spec.existing_prim_path != expected:
            raise ValueError(f"{spec.name}: unexpected laboratory camera binding")
        old_camera = world(expected)
        target = world(parent) @ pose_transform(spec.optical_pose_wxyz)
        delta = target @ np.linalg.inv(old_camera)
        if spec.parent != "world":
            assembly = parent + "/WristSensor"
            set_world(assembly, delta @ world(assembly))
        else:
            stand = "/World/Furniture/ThirdCameraStand"
            head = stand + "/Head"
            if stage.GetPrimAtPath(head):
                old_head = world(head)
                shift = target[:3, 3] - old_camera[:3, 3]
                stand_pose = world(stand)
                stand_pose[:2, 3] += shift[:2]
                set_world(stand, stand_pose)
                riser_path = stand + "/Riser"
                if stage.GetPrimAtPath(riser_path):
                    riser = UsdGeom.Cylinder(stage.GetPrimAtPath(riser_path))
                    height = riser.GetHeightAttr().Get() + shift[2]
                    if height <= 0:
                        raise ValueError("Requested camera height exceeds the tripod riser adjustment")
                    riser.GetHeightAttr().Set(float(height))
                    riser_pose = world(riser_path)
                    riser_pose[2, 3] += shift[2] / 2
                    set_world(riser_path, riser_pose)
                set_world(head, delta @ old_head)
            set_world(expected, target)
        changes[spec.name] = {"original_world_pose_wxyz": transform_pose(old_camera),
                              "configured_parent_pose_wxyz": spec.optical_pose_wxyz,
                              "source": spec.pose_source, "layer": "session"}
    return changes


def validate_robot_binding(stage, profile):
    """Do not silently move imported robots to a legacy profile's base poses."""
    import numpy as np
    from pxr import UsdGeom

    from ..robots.transforms import pose_transform

    cache = UsdGeom.XformCache()
    for mount in profile.mounts:
        expected = f"/World/Robots/R5_{mount.name.title()}"
        if mount.prim_path != expected:
            raise ValueError("tuojing_lab_v4 requires configs/robots/r5_tuojing_lab.toml")
        root = stage.GetPrimAtPath(expected)
        if not root:
            raise ValueError(f"Missing laboratory arm: {expected}")
        actual = np.array(cache.GetLocalToWorldTransform(root)).T
        if not np.allclose(actual, pose_transform(mount.base_pose_wxyz), atol=1e-6):
            raise ValueError(f"{mount.name}: profile base pose differs from the laboratory USD")
