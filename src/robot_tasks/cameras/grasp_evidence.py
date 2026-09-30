"""Conservative grasp evidence from visible RGB-D surfaces and measured TCPs.

No segmentation IDs, contacts, object poses or task scores are consumed. This
checks co-moving visible surfaces inside an approximate R5 finger cavity; it
does not certify object identity, mechanical stability or benchmark success.
"""

from dataclasses import dataclass

import numpy as np

from ..backends.scene_geometry import TCP_MARKER_RADIUS_M
from ..robots.transforms import pose_transform
from ..types import RGBFrame
from .depth import depth_array
from .geometry import project_world

CAVITY_X_M = 0.035  # Along the fingers, about the configured distal TCP.
CAVITY_Z_M = 0.025
FINGER_MARGIN_M = 0.003
MATCH_DISTANCE_M = 0.006
MIN_MATCHES = 6


@dataclass(frozen=True)
class Surface:
    points: np.ndarray  # TCP-local coordinates, metres.
    colours: np.ndarray
    camera: str


def _rays(u, v, calibration):
    fx, fy, cx, cy = calibration["intrinsics_fx_fy_cx_cy"]
    xd, yd = (u - cx) / fx, (v - cy) / fy
    x, y = xd.copy(), yd.copy()
    coefficients = list(calibration.get("distortion", ())) + [0.0] * 8
    k1, k2, p1, p2, k3, k4, k5, k6 = coefficients[:8]
    for _ in range(20 if any(coefficients) else 0):
        r2 = x * x + y * y
        radial = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2**3) / (
            1 + k4 * r2 + k5 * r2 * r2 + k6 * r2**3
        )
        x, y = (
            (xd - 2 * p1 * x * y - p2 * (r2 + 2 * x * x)) / radial,
            (yd - p1 * (r2 + 2 * y * y) - 2 * p2 * x * y) / radial,
        )
    return np.column_stack((x, -y, -np.ones_like(x)))  # USD camera axes.


def visible_surfaces(
    frames: tuple[RGBFrame, ...], tcp_pose, width_m: float, table_z_m: float
) -> tuple[Surface, ...]:
    """Only points strictly inside the jaw gap, above the known bench surface.

    The bench height is already disclosed in robot operating notes. The cavity
    is robot geometry, not a task-specific object box. Invalid/occluded depth
    yields no evidence, never a positive grasp verdict.
    """
    half_width = width_m / 2 - FINGER_MARGIN_M
    if half_width <= 0:
        return ()
    tcp = pose_transform(tcp_pose)
    bounds = np.array([CAVITY_X_M, half_width, CAVITY_Z_M])
    corners = (
        np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        * bounds
    )
    world_corners = corners @ tcp[:3, :3].T + tcp[:3, 3]
    output = []
    for frame in frames:
        depth = depth_array(frame)
        calibration = frame.calibration
        uv = project_world(
            world_corners,
            calibration["T_world_camera_pose_wxyz"],
            calibration["intrinsics_fx_fy_cx_cy"],
            calibration["distortion"],
        )
        if not np.isfinite(uv).all():
            continue
        low = np.maximum(np.floor(uv.min(axis=0)).astype(int) - 2, 0)
        high = np.minimum(
            np.ceil(uv.max(axis=0)).astype(int) + 3, [frame.width, frame.height]
        )
        if np.any(high <= low):
            continue
        v, u = np.mgrid[low[1] : high[1], low[0] : high[0]]
        u, v = u.ravel(), v.ravel()
        z = depth[v, u]
        valid = np.isfinite(z)
        u, v, z = u[valid], v[valid], z[valid]
        camera = pose_transform(calibration["T_world_camera_pose_wxyz"])
        world = (_rays(u, v, calibration) * z[:, None]) @ camera[:3, :3].T + camera[
            :3, 3
        ]
        local = (world - tcp[:3, 3]) @ tcp[:3, :3]
        inside = (np.abs(local) < bounds).all(axis=1) & (
            world[:, 2] > table_z_m + 0.006
        )
        # The rendered cyan TCP sphere moves with the robot even during an
        # empty lift. Exclude its known geometry, never count it as an object.
        inside &= np.linalg.norm(local, axis=1) > TCP_MARKER_RADIUS_M + 0.002
        points = local[inside]
        rgb = np.frombuffer(frame.data, dtype=np.uint8).reshape(
            frame.height, frame.width, 3
        )
        colours = rgb[v[inside], u[inside]].astype(float)
        # Count surface support rather than repeated pixels at high resolution.
        if len(points):
            _, indices = np.unique(
                np.floor(points / 0.003).astype(int), axis=0, return_index=True
            )
            indices = indices[
                np.linspace(0, len(indices) - 1, min(512, len(indices))).astype(int)
            ]
            output.append(Surface(points[indices], colours[indices], frame.camera))
    return tuple(output)


def match_surfaces(
    reference: tuple[Surface, ...], current: tuple[Surface, ...], width_m: float
) -> dict:
    """Compare measured geometry AND appearance in the moving TCP frame."""
    if not reference or not current:
        return {"supported": False, "matching_points": 0, "supporting_cameras": []}
    ref_points = np.concatenate([s.points for s in reference])
    ref_colours = np.concatenate([s.colours for s in reference])
    mask = np.abs(ref_points[:, 1]) < width_m / 2 - FINGER_MARGIN_M
    ref_points, ref_colours = ref_points[mask], ref_colours[mask]
    count, cameras = 0, []
    if not len(ref_points):
        return {"supported": False, "matching_points": 0, "supporting_cameras": []}
    for sample in current:
        matches = []
        for start in range(0, len(sample.points), 128):
            pts, rgb = (
                sample.points[start : start + 128],
                sample.colours[start : start + 128],
            )
            distance = np.linalg.norm(pts[:, None] - ref_points[None, :], axis=2)
            colour = np.linalg.norm(rgb[:, None] - ref_colours[None, :], axis=2)
            matches.extend(
                np.any((distance < MATCH_DISTANCE_M) & (colour < 65.0), axis=1)
            )
        matched = int(np.count_nonzero(matches))
        # A few accidental matches in a large mismatching surface are insufficient.
        if matched >= MIN_MATCHES and matched / len(sample.points) >= 0.35:
            cameras.append(sample.camera)
            count += matched
    return {
        "supported": bool(cameras),
        "matching_points": count,
        "supporting_cameras": cameras,
    }


def visible_free_points(frames: tuple[RGBFrame, ...], world_points: np.ndarray) -> int:
    """Count points whose locations were visibly empty in another capture.

    Requiring this change prevents a stationary uniform cylinder/plane from
    matching itself after a TCP lift. Occlusion or invalid depth is unknown.
    """
    if not len(world_points):
        return 0
    free = np.zeros(len(world_points), dtype=bool)
    for frame in frames:
        values = depth_array(frame)
        c = frame.calibration
        camera = pose_transform(c["T_world_camera_pose_wxyz"])
        local = (world_points - camera[:3, 3]) @ camera[:3, :3]
        expected_z = -local[:, 2]
        uv = project_world(
            world_points,
            c["T_world_camera_pose_wxyz"],
            c["intrinsics_fx_fy_cx_cy"],
            c["distortion"],
        )
        finite = np.isfinite(uv).all(axis=1) & (expected_z > 0)
        indices = np.flatnonzero(finite)
        pixels = np.rint(uv[indices]).astype(int)
        valid = (pixels >= 1).all(axis=1) & (
            pixels < [frame.width - 1, frame.height - 1]
        ).all(axis=1)
        indices, pixels = indices[valid], pixels[valid]
        # A single rounded silhouette pixel can sample background while the
        # surface is stationary, especially when the observation camera moves.
        neighbourhood = np.stack(
            [
                values[pixels[:, 1] + dy, pixels[:, 0] + dx]
                for dx in (-1, 0, 1)
                for dy in (-1, 0, 1)
            ]
        )
        free[indices] |= np.isfinite(neighbourhood).all(axis=0) & (
            neighbourhood.min(axis=0) > expected_z[indices] + 0.008
        )
    return int(np.count_nonzero(free))


def changed_surface_support(
    reference, current, before_frames, after_frames, before_pose, after_pose
) -> dict:
    def world(surfaces, pose):
        if not surfaces:
            return np.empty((0, 3))
        transform = pose_transform(pose)
        pts = np.concatenate([s.points for s in surfaces])
        return pts @ transform[:3, :3].T + transform[:3, 3]

    new = visible_free_points(before_frames, world(current, after_pose))
    vacated = visible_free_points(after_frames, world(reference, before_pose))
    return {
        "newly_occupied_points": new,
        "vacated_points": vacated,
        "visible_motion_supported": new >= MIN_MATCHES and vacated >= MIN_MATCHES,
    }
