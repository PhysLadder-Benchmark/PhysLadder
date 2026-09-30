"""L1 task adapter. No scorer output or object truth is exposed to GPT."""

from dataclasses import replace
import math

from .evaluation import L1Evaluator, L1Settings
from .specs import build_design


class L1Task:
    @staticmethod
    def prepare_config(config):
        if config.arms != ("left",):
            raise ValueError("L1 v1 uses the left R5 arm; mirrored layouts require a new task version")
        cfg = L1Settings.from_config(config)
        d = build_design(config.task_id)
        if d.insertion and {"position_tolerance_m", "height_tolerance_m"} & config.raw.get("l1", {}).keys():
            raise ValueError("Wide-clearance L1-01 uses socket containment/depth; remove legacy position/height tolerances")
        return replace(
            config,
            instruction=d.instruction
            + "\n"
            + config.instruction.format(
                stability_s=cfg.stability_s, lift_m=cfg.lift_m, grasp_hold_s=cfg.grasp_hold_s,
                retreat_m=cfg.retreat_m, axis_tilt_deg=math.degrees(cfg.rotation_tolerance_rad),
            ),
        )

    def build_scene(self, backend, config):
        from .scene import L1Scene

        self.settings = L1Settings.from_config(config)
        self.scene = L1Scene(backend, config, self.settings)
        self.arm = config.arms[0]
        self.fixed_arms = config.fixed_arms

    def reset(self, backend, seed):
        self.scene.reset(seed)
        self.scorer = L1Evaluator(self.scene.design, self.settings, self.scene.initial)
        self.initial_robot = None
        self.inactive_drift = 0.0

    def evaluate(self, state, feedback):
        robot = state.values["robot"]
        if self.initial_robot is None:
            self.initial_robot = robot
        self.inactive_drift = max(
            self.inactive_drift,
            max(
                (
                    abs(q - original)
                    for name, arm in robot.items()
                    if name in self.fixed_arms
                    for q, original in zip(
                        arm.joint_positions_rad, self.initial_robot[name].joint_positions_rad
                    )
                ),
                default=0.0,
            ),
        )
        active = robot[self.arm]
        return self.scorer.update(
            state.values["simulation_time_s"],
            self.scene.measure(),
            tcp=active.tcp_pose_world_wxyz,
            gripper_width=active.gripper_width_m,
            inactive_drift=self.inactive_drift,
        )
