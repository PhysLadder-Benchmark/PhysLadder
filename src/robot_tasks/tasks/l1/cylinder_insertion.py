"""Geometry contract for the wide-clearance L1 peg; no simulator dependencies.

Dimensions are shared by scene construction, public task facts and scoring.
The pose origin is the cylinder midpoint. Both flat ends are interchangeable.
"""

from dataclasses import dataclass
import math

import numpy as np

from ...robots.transforms import pose_transform


@dataclass(frozen=True)
class CylinderInsertion:
    radius_m: float = 0.0175
    height_m: float = 0.060
    hole_radius_m: float = 0.0275
    hole_depth_m: float = 0.020
    floor_thickness_m: float = 0.005
    wall_thickness_m: float = 0.012
    wall_segments: int = 32
    required_depth_m: float = 0.015
    geometry_tolerance_m: float = 0.0005

    @property
    def rim_z_m(self):
        return self.floor_thickness_m + self.hole_depth_m

    def measure(self, object_pose, socket_xy, max_tilt_rad):
        """Check occupied geometry, not distance to one ideal centre pose.

        The projected cylinder envelope is bounded by its lower end and rim-axis
        intersections, expanded by the tilted cross-section radius. This is
        slightly conservative for tilt, and invariant to rotation about the shaft.
        """
        t = pose_transform(object_pose)
        axis = t[:3, 2]
        # End-for-end reversal has identical geometry for a uniform cylinder.
        if axis[2] < 0:
            axis = -axis
        tip = t[:3, 3] - self.height_m / 2 * axis
        top = t[:3, 3] + self.height_m / 2 * axis
        tilt = math.acos(float(np.clip(axis[2], -1, 1)))
        depth = float(self.rim_z_m - tip[2])
        eps = self.geometry_tolerance_m
        tilted_radius_z = self.radius_m * math.sqrt(max(0., 1. - axis[2] ** 2))
        floor_clear = tip[2] - tilted_radius_z >= self.floor_thickness_m - eps
        exposed = top[2] - tilted_radius_z >= self.rim_z_m - eps
        inside = False
        radial_extent = None
        if axis[2] > 1e-6:
            rim_axis = tip + depth / axis[2] * axis
            radial_extent = float(max(np.linalg.norm(tip[:2] - socket_xy),
                                      np.linalg.norm(rim_axis[:2] - socket_xy))
                                  + self.radius_m / axis[2])
            inside = radial_extent <= self.hole_radius_m + eps
        geometry_met = bool(inside and floor_clear and exposed
                            and tilt <= max_tilt_rad and depth >= self.required_depth_m - eps)
        return {"insertion_depth_m": depth, "required_depth_m": self.required_depth_m,
                "axis_tilt_rad": tilt, "peg_radial_extent_m": radial_extent,
                "peg_inside_bore": bool(inside), "above_socket_floor": bool(floor_clear),
                "extends_above_rim": bool(exposed), "insertion_geometry_met": geometry_met}
