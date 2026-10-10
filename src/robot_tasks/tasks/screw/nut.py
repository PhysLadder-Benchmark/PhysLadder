"""Native, pre-loosened nut removal. No scripted manipulation or thread joint."""

import math
from dataclasses import dataclass, replace

import numpy as np

from ...robots.transforms import pose_transform, transform_pose
from ...types import Assessment, InvalidSimulationState
from ..bench.geometry import design
from ..bench.spec import BenchSpec
from ..l1.specs import Article, pose


def nut_spec():
    # The assembled 94 mm span is centred between the two R5 bases.
    # Native +Z points along world +X: body image-left, nut image-right.
    base = pose(0.19565 - 0.047, 1.30, 0.790, pitch=math.pi / 2)
    lift = 0.0125  # Ten whole RH turns; 2.5 mm / two turns remain below the tip.
    nut_pose = transform_pose(
        pose_transform(base) @ pose_transform(pose(0, 0, 0.075 + lift))
    )
    articles = (
        Article(
            "anchor",
            (),
            base,
            base,
            0.0563,
            asset="tuojing:anchor_removable_nut",
            asset_bounds=((-0.009, -0.009, 0), (0.009, 0.009, 0.09)),
        ),
        Article(
            "nut",
            (),
            nut_pose,
            nut_pose,
            0.0065,
            asset="tuojing:anchor_removable_nut",
            asset_bounds=((-0.0065, -0.007506, 0), (0.0065, 0.007506, 0.0065)),
        ),
    )
    return BenchSpec(
        replace(
            design("L3-05", "双臂旋下预松螺母", articles),
            version="l3-05-native-horizontal-loose-nut-v2",
            release=False,
            provenance="2026-10-08 delivered independent anchor/nut with SDF thread contact",
        ),
        arms=("left", "right"),
        mechanism="native_bimanual_sdf_nut_removal",
        limitations="Native 90 mm anchor and 13 mm across-flats / 6.5 mm tall nut. Photo-estimated M8-like 1.25 mm RH pitch, no tight preload or seal, no material damage. Preloosened ten turns / 12.5 mm, about two turns remain. CPU PhysX at 1920 Hz; no guide or fixed body in the task. Small-grasp profile allows 10 mm closure within existing joint limits; session-layer convex-decomposed R5 finger collisions preserve the pinch recess. Not hardware calibration.",
        facts="Both hands manipulate. LEFT picks up and holds the lower 12 mm diameter sleeve; RIGHT grips the opposite FLAT faces of the 13 mm hex nut (6.5 mm high), not the exposed 8 mm shaft. Native anchor length is 90 mm. The nut is loose but still threaded, about two turns remain: counterclockwise viewed from the exposed threaded end moves it outward with 1.25 mm rise per revolution. Start lying horizontally at the centre between both arms, body toward overview-image LEFT and nut toward RIGHT; lift and establish clearance between both hands and wrist cameras before turning. Regrasp as needed; never pull a still-engaged nut straight off. Fixed drives, no torque-control API. Actual detachment after bimanual turning completes the task; you may keep both pieces in the hands. No prescribed parking pose, release, extra waiting time or exact turn quota is required.",
        parameters={
            "table_z": 0.78,
            "thread_pitch_m": 0.00125,
            "shaft_tip_z": 0.09,
            "initial_nut_bottom_z": 0.0875,
            "nut_height_m": 0.0065,
            "body_arm": "left",
            "nut_arm": "right",
            "hold_s": 0.0,
            "native_instances": [
                {
                    "name": "anchor_assembly",
                    "asset": "anchor_removable_nut",
                    "pose": base,
                    "tracked": {"anchor": "AnchorBody", "nut": "Nut"},
                    "initial_body_transforms": {
                        "Nut": pose(0, 0, lift, yaw=20 * math.pi)
                    },
                }
            ],
        },
    )


def gripped(row, arm):
    return min(row["fingers"][arm]) > 0.005


@dataclass
class NutProgress:
    previous: tuple | None = None
    net_turn: float = 0.0
    net_rise: float = 0.0

    def update(self, yaw, z, controlled):
        if self.previous is not None:
            angle, height, was_controlled = self.previous
            turn = math.atan2(math.sin(yaw - angle), math.cos(yaw - angle))
            # Only measured relative motion while BOTH parts are gripped is evidence.
            # Translation/rotation of the whole assembly gives zero relative progress.
            if controlled and was_controlled and abs(turn) < math.pi / 2:
                self.net_turn += turn
                self.net_rise += z - height
        self.previous = (yaw, z, controlled)


class NutEvaluator:
    def __init__(self, spec):
        self.spec = spec
        self.progress = NutProgress()
        self.last_time = None
        self.pickup_seen = False
        self.removal_seen = False

    def update(self, now, rows):
        if not math.isfinite(now) or (
            self.last_time is not None and now <= self.last_time
        ):
            raise InvalidSimulationState(
                "Nut measurements require increasing finite time"
            )
        self.last_time = now
        body, nut = rows["anchor"], rows["nut"]
        for row in rows.values():
            values = [
                *row["pose"],
                *row["linear_velocity"],
                *row["angular_velocity"],
                row["bench_force_n"],
                *row["pair_forces"].values(),
                *[v for pair in row["fingers"].values() for v in pair],
                *np.asarray(row["corners"]).ravel(),
            ]
            if not np.isfinite(values).all():
                raise InvalidSimulationState("Nonfinite native nut measurement")
        bt, nt = pose_transform(body["pose"]), pose_transform(nut["pose"])
        relative = np.linalg.inv(bt) @ nt
        local = (np.asarray(nut["corners"]) - bt[:3, 3]) @ bt[:3, :3]
        body_held, nut_held = gripped(body, "left"), gripped(nut, "right")
        if (
            body_held
            and min(c[2] for c in body["corners"])
            > self.spec.parameters["table_z"] + 0.005
        ):
            self.pickup_seen = True
        # Nut's entire bound must clear the shaft end, in the CURRENT body frame.
        gap = float(local[:, 2].min() - self.spec.parameters["shaft_tip_z"])
        contact = float(nut["pair_forces"].get("anchor", 0.0))
        detached = gap > 0.0003 and contact < 0.005
        yaw = math.atan2(relative[1, 0], relative[0, 0])
        self.progress.update(
            yaw, float(relative[2, 3]), body_held and nut_held and self.pickup_seen
        )
        # Noise floor for a rotation/rise witness, not a prescribed number of turns.
        turned = (
            self.progress.net_turn > math.radians(2)
            and self.progress.net_rise > 0.00005
        )
        if detached and body_held and nut_held and self.pickup_seen and turned:
            self.removal_seen = True
        complete = detached and self.removal_seen and body_held and nut_held
        fallen = [
            n
            for n, r in rows.items()
            if min(c[2] for c in r["corners"]) < self.spec.parameters["table_z"] - 0.08
        ]
        metrics = {
            "task_version": self.spec.design.version,
            "mechanism": self.spec.mechanism,
            "limitations": self.spec.limitations,
            "objects": rows,
            "criteria": {
                "body_picked_up": self.pickup_seen,
                "body_held": body_held,
                "nut_held": nut_held,
                "relative_turn_rad": self.progress.net_turn,
                "relative_rise_m": self.progress.net_rise,
                "rotation_witness": turned,
                "tip_clearance_m": gap,
                "thread_contact_n": contact,
                "nut_detached": detached,
                "bimanual_removal_observed": self.removal_seen,
                "release_required": False,
                "timed_hold_s": 0,
            },
            "goal_complete": bool(complete),
            "process_constraints_satisfied": not fallen,
        }
        if fallen:
            return Assessment(
                "failed", "Object fell below the table: " + ", ".join(fallen), metrics
            )
        return Assessment(
            "succeeded" if complete else "running",
            (
                "Nut removed while both hands hold the separated parts"
                if complete
                else "Waiting for bimanual pickup and nut removal"
            ),
            metrics,
        )


class NutTask:
    @staticmethod
    def prepare_config(config):
        if config.arms != ("left", "right") or config.observation_arms:
            raise ValueError(
                "L3-05 nut removal requires both manipulation arms and no observation-only arm"
            )
        if config.physics_dt_s > 1 / 1920 + 1e-12:
            raise ValueError("Native fine threads require physics_dt_s <= 1/1920")
        if config.robot.gripper.width_limits_m[0] > 0.010:
            raise ValueError("Native 12 mm sleeve needs the small-grasp profile")
        if config.raw.get("simulation", {}).get("environment") != "tuojing_lab_v4":
            raise ValueError("Native nut task requires the Tuojing laboratory scene")
        return replace(
            config,
            instruction=config.instruction + "\n" + nut_spec().facts,
            grasp_check_mode="contact_inspection",
            initial_state_description="Both hands start empty; the assembly lies horizontally at the centre between the arms, body on overview-image LEFT and nut on RIGHT; nut pre-loosened but still threaded. No robot attachments or fixed prop bodies.",
        )

    def build_scene(self, backend, config):
        from ..bench.native_l2 import NativeL2Scene

        self.scene = NativeL2Scene(backend, config, nut_spec())

    def reset(self, backend, seed):
        self.scene.reset(seed)
        self.evaluator = NutEvaluator(self.scene.spec)
        self._last_time = self._assessment = None

    def evaluate(self, state, feedback):
        now = state.values["simulation_time_s"]
        if now != self._last_time:
            self._assessment = self.evaluator.update(now, self.scene.measure())
            self._last_time = now
        return self._assessment
