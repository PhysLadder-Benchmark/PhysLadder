"""Shared convex D-block geometry and shallow-recess scoring, in metres."""

import math
from dataclasses import dataclass

import numpy as np

from ...robots.transforms import pose_transform


def offset_polygon(points, distance):
    """Offset the supporting lines of a counterclockwise convex polygon."""
    points = np.asarray(points)
    edges = np.roll(points, -1, axis=0) - points
    normals = np.column_stack((edges[:, 1], -edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    bounds = np.sum(normals * points, axis=1) + distance
    return np.array(
        [
            np.linalg.solve(normals[[i - 1, i]], bounds[[i - 1, i]])
            for i in range(len(points))
        ]
    )


def loft(lower, upper, bottom, top):
    """Closed outward-facing mesh between corresponding CCW planar rings."""
    n = len(lower)
    vertices = tuple(
        (float(x), float(y), float(z))
        for ring, z in ((lower, bottom), (upper, top))
        for x, y in ring
    )
    faces = [(0, i + 1, i) for i in range(1, n - 1)]
    faces += [(n, n + i, n + i + 1) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    return vertices, tuple(faces)


@dataclass(frozen=True)
class ShapedRecess:
    radius_m: float = 0.025
    flat_x_m: float = -0.008
    height_m: float = 0.040
    clearance_m: float = 0.004
    hole_depth_m: float = 0.012
    floor_thickness_m: float = 0.006
    wall_thickness_m: float = 0.010
    chamfer_height_m: float = 0.003
    chamfer_extra_m: float = 0.002
    required_depth_m: float = 0.010
    geometry_tolerance_m: float = 0.0005
    arc_segments: int = 32

    @property
    def rim_z_m(self):
        return self.floor_thickness_m + self.hole_depth_m

    @property
    def profile(self):
        angle = math.acos(self.flat_x_m / self.radius_m)
        angles = np.linspace(-angle, angle, self.arc_segments + 1)
        return self.radius_m * np.column_stack((np.cos(angles), np.sin(angles)))

    def block_mesh(self):
        return loft(self.profile, self.profile, -self.height_m / 2, self.height_m / 2)

    def wall_meshes(self):
        inner = offset_polygon(self.profile, self.clearance_m)
        mouth = offset_polygon(self.profile, self.clearance_m + self.chamfer_extra_m)
        outer = offset_polygon(self.profile, self.clearance_m + self.wall_thickness_m)
        shoulder = self.rim_z_m - self.chamfer_height_m
        for i in range(len(inner)):
            j = (i + 1) % len(inner)
            lower = np.array([inner[i], outer[i], outer[j], inner[j]])
            upper = np.array([mouth[i], outer[i], outer[j], mouth[j]])
            yield loft(lower, lower, self.floor_thickness_m, shoulder)
            yield loft(lower, upper, shoulder, self.rim_z_m)

    def measure(self, object_pose, socket_xy, max_tilt_rad):
        transform = pose_transform(object_pose)
        vertices, faces = self.block_mesh()
        world = np.asarray(vertices) @ transform[:3, :3].T + transform[:3, 3]
        n = len(self.profile)
        bottom = world[:n] if transform[2, 2] >= 0 else world[n:]
        tilt = math.acos(float(np.clip(abs(transform[2, 2]), 0, 1)))
        depth = float(self.rim_z_m - bottom[:, 2].max())
        eps = self.geometry_tolerance_m
        floor_clear = bool(bottom[:, 2].min() >= self.floor_thickness_m - eps)
        # Clip the real convex solid at the rim, including edge intersections.
        occupied = [v for v in world if v[2] <= self.rim_z_m]
        edges = {
            tuple(sorted((a, b)))
            for face in faces
            for a, b in zip(face, (*face[1:], face[0]))
        }
        for a, b in edges:
            va, vb = world[a], world[b]
            if (va[2] - self.rim_z_m) * (vb[2] - self.rim_z_m) < 0:
                occupied.append(
                    va + (vb - va) * (self.rim_z_m - va[2]) / (vb[2] - va[2])
                )
        inside = False
        if occupied:
            occupied = np.asarray(occupied)
            profile = self.profile
            edges_xy = np.roll(profile, -1, axis=0) - profile
            normals = np.column_stack((edges_xy[:, 1], -edges_xy[:, 0]))
            normals /= np.linalg.norm(normals, axis=1)[:, None]
            bounds = np.sum(normals * profile, axis=1)
            bevel = self.chamfer_extra_m * np.clip(
                (occupied[:, 2] - (self.rim_z_m - self.chamfer_height_m))
                / self.chamfer_height_m,
                0,
                1,
            )
            excess = (occupied[:, :2] - socket_xy) @ normals.T - bounds
            inside = bool(np.all(excess <= (self.clearance_m + bevel + eps)[:, None]))
        geometry_met = bool(
            inside
            and floor_clear
            and tilt <= max_tilt_rad
            and depth >= self.required_depth_m - eps
        )
        return {
            "insertion_depth_m": depth,
            "required_depth_m": self.required_depth_m,
            "axis_tilt_rad": tilt,
            "block_inside_recess": inside,
            "above_socket_floor": floor_clear,
            "insertion_geometry_met": geometry_met,
        }
