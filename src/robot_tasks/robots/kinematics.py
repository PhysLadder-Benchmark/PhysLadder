"""URDF-derived R5 FK and bounded local IK; no simulator or ROS dependency.

IK chooses a joint goal, not a collision-free or straight Cartesian trajectory.
A local solve that does not converge is not proof of global unreachability.
"""

from dataclasses import dataclass

import numpy as np

from .profile import RobotProfile
from .transforms import (axis_rotation, finite_vector, origin_transform, pose_residual,
                         pose_transform, transform_pose)
from .urdf import read_urdf


@dataclass(frozen=True)
class IKResult:
    converged: bool
    positions_rad: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    reason: str


class IKError(ValueError):
    def __init__(self, result: IKResult):
        self.result = result
        super().__init__(f"IK rejected: {result.reason}; position error={result.position_error_m:.6f} m, "
                         f"orientation error={result.orientation_error_rad:.6f} rad")


class R5Kinematics:
    position_tolerance_m = 1e-4
    orientation_tolerance_rad = 1e-3

    def __init__(self, profile: RobotProfile):
        if profile.tcp_pose_wxyz is None or not profile.tcp_source:
            raise ValueError("Cartesian control requires an explicitly defined TCP and source")
        self.profile = profile
        description = read_urdf(profile.asset_path, dict(profile.package_roots))
        if description.chain(profile.base_link, profile.ee_link) != profile.joint_names:
            raise ValueError("IK joint order must match the URDF chain")
        by_child = {joint.child: joint for joint in description.joints}
        chain, tip = [], profile.ee_link
        while tip != profile.base_link:
            joint = by_child[tip]
            if joint.kind not in {"fixed", "revolute", "continuous"} or joint.mimic is not None:
                raise ValueError("R5 IK supports independent rotational arm joints and fixed links")
            chain.append(joint)
            tip = joint.parent
        self.chain = tuple(reversed(chain))
        self.origins = tuple(origin_transform(j.origin_xyz, j.origin_rpy) for j in self.chain)
        self.tcp = pose_transform(profile.tcp_pose_wxyz)
        self.bases = {mount.name: pose_transform(mount.base_pose_wxyz) for mount in profile.mounts
                      if mount.base_pose_wxyz is not None}
        limits = {bound.joint: bound for bound in profile.operating_limits}
        if not set(profile.joint_names).issubset(limits):
            raise ValueError("IK requires configured arm operating limits")
        self.lower = np.array([limits[name].lower for name in profile.joint_names])
        self.upper = np.array([limits[name].upper for name in profile.joint_names])
        self.reach_bound_m = sum(np.linalg.norm(t[:3, 3]) for t in self.origins) + np.linalg.norm(self.tcp[:3, 3])

    def _forward_jacobian(self, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        transform, axes, points = np.eye(4), [], []
        index = 0
        for joint, origin in zip(self.chain, self.origins):
            transform = transform @ origin
            if joint.kind != "fixed":
                axis = np.asarray(joint.axis) / np.linalg.norm(joint.axis)
                axes.append(transform[:3, :3] @ axis)
                points.append(transform[:3, 3].copy())
                motion = np.eye(4)
                motion[:3, :3] = axis_rotation(axis, positions[index])
                transform = transform @ motion
                index += 1
        transform = transform @ self.tcp
        jacobian = np.array([np.r_[np.cross(axis, transform[:3, 3] - point), axis]
                             for point, axis in zip(points, axes)]).T
        return transform, jacobian

    def forward(self, positions, arm: str, frame: str = "world") -> tuple[float, ...]:
        q = finite_vector(positions, len(self.profile.joint_names), "Joint positions")
        transform, _ = self._forward_jacobian(q)
        self._check_frame(arm, frame)
        return transform_pose(self.bases[arm] @ transform if frame == "world" else transform)

    def _check_frame(self, arm: str, frame: str) -> None:
        if frame not in {"world", "robot_base"}:
            raise ValueError("Cartesian frame must be world or robot_base")
        if arm not in self.bases:
            raise ValueError(f"No configured base pose for arm: {arm}")

    def world_pose(self, pose, arm: str, frame: str) -> tuple[float, ...]:
        self._check_frame(arm, frame)
        target = pose_transform(pose)
        return transform_pose(self.bases[arm] @ target if frame == "robot_base" else target)

    def solve(self, pose, arm: str, seed, frame: str = "world", *, max_iterations: int = 120) -> IKResult:
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError("IK max_iterations must be a positive integer")
        target = pose_transform(self.world_pose(pose, arm, frame))
        target = np.linalg.inv(self.bases[arm]) @ target
        q = finite_vector(seed, len(self.profile.joint_names), "IK seed")
        # Measured physics states can lie slightly outside nominal limits.
        q = np.clip(q, self.lower, self.upper)
        weights = np.array([1., 1., 1., 0.15, 0.15, 0.15])
        reason = "local_solver_not_converged"
        iteration = 0
        for iteration in range(max_iterations + 1):
            current, jacobian = self._forward_jacobian(q)
            error = pose_residual(current, target)
            position_error, orientation_error = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
            if position_error <= self.position_tolerance_m and orientation_error <= self.orientation_tolerance_rad:
                return IKResult(True, tuple(float(v) for v in q), float(position_error),
                                float(orientation_error), iteration, "converged")
            if np.linalg.norm(target[:3, 3]) > self.reach_bound_m + self.position_tolerance_m:
                reason = "outside_conservative_reach_bound"
                break
            if iteration == max_iterations:
                break
            weighted_jacobian, weighted_error = weights[:, None] * jacobian, weights * error
            delta = weighted_jacobian.T @ np.linalg.solve(
                weighted_jacobian @ weighted_jacobian.T + 0.01**2 * np.eye(6), weighted_error)
            delta *= min(1.0, 0.15 / max(float(np.max(np.abs(delta))), 1e-12))
            improved = False
            for fraction in (1., .5, .25, .125, .0625):
                candidate = np.clip(q + fraction * delta, self.lower, self.upper)
                next_pose, _ = self._forward_jacobian(candidate)
                if np.linalg.norm(weights * pose_residual(next_pose, target)) < np.linalg.norm(weighted_error) - 1e-12:
                    q, improved = candidate, True
                    break
            if not improved:
                break
        # Failed solutions are diagnostic only; callers must never execute them.
        return IKResult(False, tuple(float(v) for v in q), float(position_error),
                        float(orientation_error), iteration, reason)
