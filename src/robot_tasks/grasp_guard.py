"""Shared observation-only grasp checkpoint; never reads evaluator state.

The model still chooses all motions. Pending grasps permit observation moves,
opening, and a small vertical test lift, but cannot proceed to transport merely
because a decision note says 'grasped'. Evidence is deliberately conservative.
"""

from dataclasses import dataclass, replace

import numpy as np

from .backends.scene_geometry import BENCH_TOP
from .cameras.grasp_evidence import (
    Surface,
    changed_surface_support,
    match_surfaces,
    visible_surfaces,
)
from .robots.transforms import pose_errors, pose_transform, transform_pose
from .types import Action, Feedback, GraspReference, Observation

MIN_LIFT_M = 0.015
MAX_TEST_LIFT_M = 0.040
MAX_TEST_XY_M = 0.005
MAX_TEST_ROTATION_RAD = 0.10
EMPTY_TOLERANCE_M = 0.0015


@dataclass
class Attempt:
    reference: GraspReference
    surfaces: tuple[Surface, ...]
    status: str = "awaiting_lift"
    reason: str = (
        "Close, then test-lift vertically 0.02-0.03 m and inspect the paired views."
    )
    opening_requested: bool = False
    verified_once: bool = False
    evidence: dict | None = None


class GraspGuard:
    def __init__(self, config):
        self.config = config
        self.attempts: dict[str, Attempt] = {}
        self.rejections = 0
        self.verifications = 0
        self._last_observation_key = None

    def reset(self):
        # Run totals remain, but no evidence survives a physical reset.
        self.attempts.clear()
        self._last_observation_key = None

    def _world_target(self, command):
        if command.frame == "world":
            return command.values
        mount = next(m for m in self.config.robot.mounts if m.name == command.arm)
        return transform_pose(
            pose_transform(mount.base_pose_wxyz) @ pose_transform(command.values)
        )

    def reject_reason(self, action: Action, observation: Observation) -> str | None:
        """Validate the entire action before planner/robot side effects."""
        closing = {
            c.arm
            for c in action.commands
            if c.kind == "gripper"
            and c.arm in self.config.arms
            and observation.robot[c.arm].gripper_width_m is not None
            and c.values[0] < observation.robot[c.arm].gripper_width_m - 0.001
        }
        for command in action.commands:
            arm = command.arm
            if arm not in self.config.arms or command.kind == "gripper":
                continue
            state = observation.robot[arm]
            attempt = self.attempts.get(arm)
            if command.kind == "joint_position":
                if arm in closing or (attempt and attempt.status != "verified"):
                    return f"{arm}: use Cartesian test lift while grasp evidence is pending; no joint-motion bypass."
                continue
            target = self._world_target(command)
            if state.tcp_pose_world_wxyz is None:
                return f"{arm}: measured TCP is missing; cannot verify grasp motion."
            position, rotation = pose_errors(state.tcp_pose_world_wxyz, target)
            if arm in closing and (position > 0.002 or rotation > 0.03):
                return f"{arm}: approach first, then close in a separate decision so the actual grasp pose is observed."
            if position <= 1e-6 and rotation <= 1e-5:
                continue  # Holding the measured pose must never prevent opening/recovery.
            if not attempt or attempt.status == "verified":
                continue
            origin = attempt.reference.robot[arm].tcp_pose_world_wxyz
            delta = np.array(target[:3]) - origin[:3]
            _, turn = pose_errors(origin, target)
            if (
                np.linalg.norm(delta[:2]) > MAX_TEST_XY_M
                or delta[2] < -0.002
                or delta[2] > MAX_TEST_LIFT_M
                or turn > MAX_TEST_ROTATION_RAD
            ):
                return (
                    f"{arm}: grasp is {attempt.status}, not verified. {attempt.reason} "
                    "Transport/insertion is blocked. Hold this arm and inspect with an observation arm, "
                    "or open in a separate action before repositioning/regrasping. "
                    "A pending grasp permits only a vertical test lift within 0.04 m of its closing pose "
                    "(at most 0.005 m horizontal drift); repeated small steps cannot bypass this bound."
                )
        return None

    def accepted(
        self, action: Action, observation: Observation
    ) -> tuple[GraspReference, ...]:
        """Capture a baseline only AFTER atomic robot.apply accepted the action."""
        references = []
        for command in action.commands:
            arm = command.arm
            if command.kind != "gripper" or arm not in self.config.arms:
                continue
            state = observation.robot[arm]
            width = state.gripper_width_m
            if width is None or state.tcp_pose_world_wxyz is None:
                continue
            if command.values[0] > width + 0.001:
                if arm in self.attempts:
                    self.attempts[arm].opening_requested = True
                    self.attempts[arm].status = "release_pending"
                continue
            if command.values[0] >= width - 0.001:
                continue  # Reissuing close must not move the reference origin.
            if arm in self.attempts:
                self.attempts[arm].opening_requested = False
                self.attempts[arm].status = "awaiting_lift"
                continue
            reference = GraspReference(
                arm, observation.time_s, observation.rgb, observation.robot
            )
            surfaces = visible_surfaces(
                observation.rgb, state.tcp_pose_world_wxyz, width, BENCH_TOP
            )
            self.attempts[arm] = Attempt(reference, surfaces)
            references.append(reference)
        self._last_observation_key = None
        return tuple(references)

    def enrich(self, observation: Observation) -> Observation:
        if not self.attempts:
            return observation
        key = (
            observation.time_s,
            tuple((f.camera, f.calibration.get("capture_id")) for f in observation.rgb),
        )
        if key != self._last_observation_key:
            self._update(observation)
            self._last_observation_key = key
        details = {
            arm: {
                "status": attempt.status,
                "reason": attempt.reason,
                "reference_time_s": attempt.reference.time_s,
                "source": "visible_rgbd_and_measured_robot_only",
                **(attempt.evidence or {}),
            }
            for arm, attempt in self.attempts.items()
        }
        feedback = replace(
            observation.feedback,
            metrics={**observation.feedback.metrics, "grasp_check": details},
        )
        references = tuple(
            a.reference for a in self.attempts.values() if a.status != "verified"
        )
        return replace(observation, feedback=feedback, grasp_references=references)

    def _update(self, observation):
        minimum, maximum = self.config.robot.gripper.width_limits_m
        for arm, attempt in list(self.attempts.items()):
            state = observation.robot[arm]
            pose, width = state.tcp_pose_world_wxyz, state.gripper_width_m
            if pose is None or width is None:
                attempt.verified_once = False
                attempt.status, attempt.reason = (
                    "unknown",
                    "Measured TCP/opening unavailable; inspect or reopen.",
                )
                continue
            before = attempt.reference.robot[arm]
            # Release only when opening is actually measured, not just commanded.
            previous_width = (attempt.evidence or {}).get("measured_width_m", width)
            if attempt.opening_requested and (
                width > previous_width + 0.002 or width >= maximum - 0.002
            ):
                del self.attempts[arm]
                continue
            rise = pose[2] - before.tcp_pose_world_wxyz[2]
            attempt.evidence = {
                "measured_width_m": width,
                "tcp_rise_m": rise,
                "matching_points": 0,
                "supporting_cameras": [],
            }
            if width <= minimum + EMPTY_TOLERANCE_M:
                attempt.verified_once = False
                attempt.status = "empty_suspected"
                attempt.reason = (
                    "Fingers reached the empty-close limit; this is not grasp evidence. "
                    "For an object wider than this measured gap, inspect its location and regrasp."
                )
                continue
            current = visible_surfaces(observation.rgb, pose, width, BENCH_TOP)
            evidence = match_surfaces(attempt.surfaces, current, width)
            if rise >= MIN_LIFT_M and not attempt.verified_once:
                evidence.update(
                    changed_surface_support(
                        attempt.surfaces,
                        current,
                        attempt.reference.rgb,
                        observation.rgb,
                        before.tcp_pose_world_wxyz,
                        pose,
                    )
                )
                evidence["supported"] &= evidence["visible_motion_supported"]
            attempt.evidence.update(evidence)
            if attempt.verified_once:
                if evidence["supported"] and not attempt.opening_requested:
                    attempt.status, attempt.reason = (
                        "verified",
                        "Visible surfaces remain consistent with the lifted object in the finger cavity.",
                    )
                else:
                    attempt.verified_once = False
                    attempt.status, attempt.reason = (
                        "unknown",
                        "Previously seen held-surface evidence is missing or occluded; inspect before continuing.",
                    )
            elif rise < MIN_LIFT_M:
                attempt.status, attempt.reason = (
                    "awaiting_lift",
                    "Not yet verified: test-lift vertically 0.02-0.03 m, then inspect.",
                )
            elif evidence["supported"] and not attempt.opening_requested:
                attempt.status, attempt.reason = (
                    "verified",
                    "RGB-D surface geometry/appearance follows the measured test lift inside the finger cavity.",
                )
                attempt.verified_once = True
                self.verifications += 1
            else:
                attempt.status, attempt.reason = (
                    "unknown",
                    "No sufficient co-moving surface evidence after lift; inspect the paired views or reopen and regrasp.",
                )

    def rejection(self, reason: str, observation: Observation) -> Observation:
        self.rejections += 1
        feedback = Feedback(
            "grasp_check_rejected",
            reason,
            {**observation.feedback.metrics, "reason_code": "grasp_not_verified"},
        )
        return replace(observation, feedback=feedback)
