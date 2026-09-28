"""Paused-render freshness and projection check using a temporary fiducial.

Only engineering checks create this marker. It is removed before returning and
is never injected into a benchmark observation or GPT prompt.
"""

import numpy as np

from ..cameras.geometry import project_world
from ..robots.transforms import pose_transform


def check_camera_projection(backend) -> dict:
    from isaacsim.core.api.objects import VisualSphere
    from pxr import Gf, Sdf, UsdShade

    path = "/World/ProjectionCheckMarker"
    sphere = VisualSphere(path, name="projection_check_marker", radius=.003,
                          position=np.array([0., 0., -1.]), color=np.array([1., 0., 1.]))
    material = UsdShade.Material.Define(backend.world.stage, path + "/EmissiveMaterial")
    shader = UsdShade.Shader.Define(backend.world.stage, path + "/EmissiveMaterial/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0., 0., 0.))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1., 0., 1.))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(sphere.prim).Bind(material)
    results = {}
    timestamp = backend.time_s()
    try:
        for spec in backend.camera_rig.specs:
            pose = backend.camera_rig.world_poses()[spec.name]
            matrix = pose_transform(pose)
            # Newly created geometry/materials need initial GPU resource upload.
            # Warm up once at the centre, then test two subsequent pose changes.
            sphere.set_world_pose(position=(matrix @ np.array([0., 0., -.08, 1.]))[:3])
            for _ in range(3):
                backend.capture_rgb()
            samples = []
            previous_pixel = None
            for x in (-.02, .02):
                point = (matrix @ np.array([x, 0., -.08, 1.]))[:3]
                sphere.set_world_pose(position=point)
                frames = backend.capture_rgb()
                frame = next(f for f in frames if f.camera == spec.name)
                expected = project_world([point], pose, spec.intrinsics, spec.distortion)[0]
                image = np.frombuffer(frame.data, np.uint8).reshape(frame.height, frame.width, 3).astype(float)
                mask = (image[:, :, 0] > 120) & (image[:, :, 2] > 120) & (image[:, :, 1] < 80)
                u, v = (int(round(a)) for a in expected)
                ys, xs = np.where(mask[max(0,v-20):v+21, max(0,u-20):u+21])
                detected = None if len(xs) < 4 else [float(xs.mean() + max(0,u-20)), float(ys.mean() + max(0,v-20))]
                error = None if detected is None else float(np.linalg.norm(np.asarray(detected) - expected))
                old_visible = False
                if previous_pixel is not None:
                    pu, pv = previous_pixel
                    old_visible = bool(mask[pv-4:pv+5, pu-4:pu+5].any())
                samples.append({"expected_uv": expected.tolist(), "detected_uv": detected,
                                "pixel_error": error, "old_marker_visible": old_visible,
                                "passed": error is not None and error < 3 and not old_visible})
                previous_pixel = (u, v)
            results[spec.name] = {"samples": samples, "passed": all(s["passed"] for s in samples)}
        unchanged = abs(backend.time_s() - timestamp) < 1e-9
        return {"cameras": results, "physics_time_unchanged": unchanged,
                "passed": unchanged and all(c["passed"] for c in results.values()),
                "scope": "Analytic camera projection versus rendered marker; marker moves with physics paused"}
    finally:
        backend.world.stage.RemovePrim(path)
        backend.capture_rgb()
