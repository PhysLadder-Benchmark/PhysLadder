"""RGB render products attached to world or measured R5 rigid-link transforms."""

import numpy as np

from ..robots.transforms import transform_pose
from ..types import RGBFrame
from .profile import CameraRigProfile
from . import RGBNotReady


def _set_pose(prim, pose):
    from pxr import Gf, UsdGeom
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(pose[3], Gf.Vec3d(*pose[4:])))


class IsaacCameraRig:
    def __init__(self, stage, rep, profile: CameraRigProfile, names: tuple[str, ...]):
        self.stage, self.rep, self.profile = stage, rep, profile
        self.specs = profile.select(names)
        self.paths, self.products, self.annotators, self.sensors = {}, {}, {}, {}

    def create(self, ee_paths: dict[str, str]) -> dict:
        from isaacsim.sensors.camera import Camera
        from pxr import UsdGeom
        report = {"profile": str(self.profile.source), "calibration_status": self.profile.calibration_status,
                  "hardware_calibrated": False, "notes": self.profile.notes, "cameras": {}}
        for spec in self.specs:
            parent = "/World" if spec.parent == "world" else ee_paths[spec.parent]
            root_path = f"{parent}/Sensors/{spec.name}"
            root = UsdGeom.Xform.Define(self.stage, root_path)
            _set_pose(root.GetPrim(), spec.optical_pose_wxyz)
            housing = UsdGeom.Xform.Define(self.stage, root_path + "/Housing")
            housing.GetPrim().GetReferences().AddReference(str(spec.asset))
            _set_pose(housing.GetPrim(), spec.asset_pose_wxyz)
            camera = UsdGeom.Camera.Define(self.stage, root_path + "/RGB")
            camera.CreateClippingRangeAttr(spec.clipping_m)
            camera.CreateFStopAttr(0.0)
            # USD camera looks down -Z. The optical root carries all mount transforms.
            sensor = Camera(str(camera.GetPath()), name=spec.name, resolution=spec.resolution)
            fx, fy, cx, cy = spec.intrinsics
            sensor.set_opencv_pinhole_properties(cx=cx, cy=cy, fx=fx, fy=fy, pinhole=list(spec.distortion))
            self.sensors[spec.name] = sensor
            self.paths[spec.name] = str(camera.GetPath())
            product = self.rep.create.render_product(self.paths[spec.name], spec.resolution)
            self.products[spec.name] = product
            annotator = self.rep.AnnotatorRegistry.get_annotator("rgb")
            self.annotators[spec.name] = annotator
            annotator.attach(product)
            report["cameras"][spec.name] = {**spec.metadata(), "parent_prim": parent,
                                           "camera_prim": self.paths[spec.name]}
        return report

    def read(self, timestamp: float) -> tuple[RGBFrame, ...]:
        frames = []
        for spec in self.specs:
            data = np.asarray(self.annotators[spec.name].get_data())
            if data.size == 0:
                raise RGBNotReady(f"{spec.name}: RGB buffer is not ready")
            width, height = spec.resolution
            if data.ndim != 3 or data.shape[:2] != (height, width) or data.shape[2] not in {3, 4}:
                raise RuntimeError(f"{spec.name}: invalid RGB dimensions {data.shape}")
            rgb = np.ascontiguousarray(data[..., :3], dtype=np.uint8)
            frames.append(RGBFrame(spec.name, timestamp, width, height, rgb.tobytes()))
        return tuple(frames)

    def world_poses(self) -> dict[str, tuple[float, ...]]:
        from pxr import Usd, UsdGeom
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        return {name: transform_pose(np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))).T)
                for name, path in self.paths.items()}

    def close(self) -> None:
        for name, annotator in self.annotators.items():
            annotator.detach(self.products[name])
        for product in self.products.values():
            product.destroy()
        for sensor in self.sensors.values():
            sensor.destroy()
        self.annotators.clear()
        self.products.clear()
        self.sensors.clear()
        self.paths.clear()
