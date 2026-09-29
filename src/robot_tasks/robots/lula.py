"""R5 IK through the NVIDIA Lula library bundled with Isaac Sim.

Uses the same compute_ik_ccd API as Isaac's LulaKinematicsSolver. All goals
refer to the configured TCP; the controller still executes joint drive targets.
There is no alternate inverse solver, trajectory planner or collision avoidance.
"""

from copy import deepcopy
import hashlib
import importlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import IKResult, R5Kinematics
from .transforms import finite_vector, pose_residual, pose_transform


TCP_FRAME = "robot_tasks_tcp"


def prepare_lula_assets(kinematics: R5Kinematics, directory: Path) -> tuple[Path, Path]:
    """Derive a mesh-free arm URDF + descriptor without modifying vendor assets.

    Fingers remain independently controlled by the existing gripper adapter.
    Their geometry is unnecessary for IK of the fixed TCP relative to ee_link.
    """
    profile = kinematics.profile
    source = ET.parse(profile.asset_path).getroot()
    joints = {joint.get("name"): joint for joint in source.findall("joint")}
    limits = {bound.joint: bound for bound in profile.operating_limits}
    robot = ET.Element("robot", name="r5_lula_kinematics")
    links = [profile.base_link] + [joint.child for joint in kinematics.chain]
    if TCP_FRAME in links:
        raise ValueError(f"Reserved Lula TCP frame already exists: {TCP_FRAME}")
    for name in links:
        ET.SubElement(robot, "link", name=name)
    for joint in kinematics.chain:
        element = deepcopy(joints[joint.name])
        if joint.kind != "fixed":
            element.set("type", "revolute")  # Respect finite simulation limits, even for continuous sources.
            bound = limits[joint.name]
            limit = element.find("limit")
            if limit is None:
                limit = ET.SubElement(element, "limit")
            for name in ("lower", "upper", "velocity", "effort"):
                limit.set(name, str(getattr(bound, name)))
        robot.append(element)
    ET.SubElement(robot, "link", name=TCP_FRAME)
    fixed = ET.SubElement(robot, "joint", name="robot_tasks_tcp_fixed", type="fixed")
    ET.SubElement(fixed, "parent", link=profile.ee_link)
    ET.SubElement(fixed, "child", link=TCP_FRAME)
    r = kinematics.tcp[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if math.hypot(r[0, 0], r[1, 0]) > 1e-9:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = math.atan2(-r[1, 2], r[1, 1]), 0.0
    ET.SubElement(fixed, "origin", xyz=" ".join(map(str, kinematics.tcp[:3, 3])),
                  rpy=" ".join(map(str, (roll, pitch, yaw))))
    urdf_bytes = ET.tostring(robot, encoding="utf-8", xml_declaration=True)
    # JSON flow sequences are valid YAML and avoid a new YAML runtime dependency.
    descriptor = ("api_version: 1.0\n"
                  f"root_link: {json.dumps(profile.base_link)}\n"
                  f"cspace: {json.dumps(profile.joint_names)}\n"
                  f"default_q: {json.dumps(np.clip(np.zeros(len(profile.joint_names)), kinematics.lower, kinematics.upper).tolist())}\n"
                  "cspace_to_urdf_rules: []\ncollision_spheres: []\n").encode("utf-8")
    identity = hashlib.sha256(urdf_bytes + descriptor).hexdigest()[:16]
    destination = directory / "lula" / identity
    destination.mkdir(parents=True, exist_ok=True)
    for name, data in (("r5.urdf", urdf_bytes), ("robot_description.yaml", descriptor)):
        path = destination / name
        if path.exists():
            if path.read_bytes() != data:
                raise ValueError(f"Generated Lula asset was modified: {path}")
        else:
            path.write_bytes(data)
    return destination / "r5.urdf", destination / "robot_description.yaml"


class R5LulaKinematics(R5Kinematics):
    """A single inverse solver, with independent FK/limit checks before actuation."""

    def __init__(self, profile, asset_directory: Path, *, sampling_seed: int = 0, lula_module=None):
        super().__init__(profile)
        try:
            self.lula = lula_module if lula_module is not None else importlib.import_module("lula")
        except ImportError as exc:
            raise RuntimeError("Lula is required: enable isaacsim.robot_motion.lula in the Isaac Python environment") from exc
        self.urdf_path, self.description_path = prepare_lula_assets(self, asset_directory)
        self.description = self.lula.load_robot(str(self.description_path), str(self.urdf_path))
        self.solver = self.description.kinematics()
        actual = tuple(self.description.c_space_coord_name(i) for i in range(self.description.num_c_space_coords()))
        if actual != profile.joint_names or self.solver.base_frame_name() != profile.base_link:
            raise RuntimeError("Lula joint order or base frame does not match R5 configuration")
        if TCP_FRAME not in self.solver.frame_names():
            raise RuntimeError("Lula model is missing the configured TCP frame")
        self.ik_config = self.lula.CyclicCoordDescentIkConfig()
        self.ik_config.position_tolerance = self.position_tolerance_m
        # Lula uses per-axis vector error. A tighter bound also satisfies the
        # existing overall angular tolerance, which is checked independently below.
        self.ik_config.orientation_tolerance = 2 * math.sin(self.orientation_tolerance_rad / (2 * math.sqrt(2)))
        self.ik_config.sampling_seed = sampling_seed
        self.validate_model()

    def validate_model(self) -> None:
        """Fail at startup if the generated model and source FK disagree."""
        seeds = [np.clip(np.zeros(len(self.lower)), self.lower, self.upper)]
        seeds += [finite_vector(m.home_positions_rad, len(self.lower), "Home joints") for m in self.profile.mounts]
        for q in seeds:
            lula_pose = np.asarray(self.solver.pose(q, TCP_FRAME).matrix())
            if not np.allclose(lula_pose, self._forward_transform(q), atol=1e-8, rtol=0):
                raise RuntimeError("Lula FK differs from the source R5 URDF/TCP definition")

    @property
    def metadata(self) -> dict:
        return {"solver": "lula", "api": "compute_ik_ccd", "tcp_frame": TCP_FRAME,
                "urdf": str(self.urdf_path), "description": str(self.description_path),
                "urdf_sha256": hashlib.sha256(self.urdf_path.read_bytes()).hexdigest(),
                "description_sha256": hashlib.sha256(self.description_path.read_bytes()).hexdigest(),
                "joint_names": self.profile.joint_names, "position_tolerance_m": self.position_tolerance_m,
                "orientation_tolerance_rad": self.orientation_tolerance_rad,
                "sampling_seed": self.ik_config.sampling_seed,
                "max_num_descents": self.ik_config.max_num_descents,
                "ccd_max_iterations": self.ik_config.ccd_max_iterations,
                "bfgs_max_iterations": self.ik_config.bfgs_max_iterations,
                "collision_avoidance": False, "fallback": None}

    def solve(self, pose, arm: str, seed, frame: str = "world") -> IKResult:
        world_target = pose_transform(self.world_pose(pose, arm, frame))
        target = np.linalg.inv(self.bases[arm]) @ world_target
        q = np.clip(finite_vector(seed, len(self.profile.joint_names), "IK seed"), self.lower, self.upper)
        reason, descents = "outside_conservative_reach_bound", 0
        success = False
        if np.linalg.norm(target[:3, 3]) <= self.reach_bound_m + self.position_tolerance_m:
            self.ik_config.cspace_seeds = [q]
            result = self.lula.compute_ik_ccd(self.solver, self.lula.Pose3(target), TCP_FRAME, self.ik_config)
            q = np.asarray(result.cspace_position, dtype=float).reshape(-1)
            if q.shape != self.lower.shape or not np.isfinite(q).all():
                raise RuntimeError("Lula returned invalid joint positions")
            success, descents = bool(result.success), int(result.num_descents)
            reason = "converged" if success else "lula_not_converged"
            if np.any(q < self.lower) or np.any(q > self.upper):
                success, reason = False, "lula_joint_limit_violation"
        error = pose_residual(self._forward_transform(q), target)
        position_error, orientation_error = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        if success and (position_error > self.position_tolerance_m or orientation_error > self.orientation_tolerance_rad):
            success, reason = False, "lula_residual_tolerance_exceeded"
        return IKResult(success, tuple(float(v) for v in q), position_error, orientation_error,
                        None, reason, descents)
