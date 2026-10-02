"""Two-level cabinet and a single-cycle push-to-open latch engineering proxy.

Latch state follows measured compression/rebound, never a robot command or
score. A bounded, unilateral spring supplies the pop-out energy. The internal
heart-cam and material deformation of commercial latches are not simulated.
"""

from dataclasses import asdict, dataclass, replace
import math

from ..l1.specs import BLUE, GREEN, GREY, RED, Article, box, pose
from .geometry import design
from .spec import BenchSpec, Joint


@dataclass(frozen=True)
class PushLatchSettings:
    compression_m: float = 0.008
    compression_hold_s: float = 0.04
    rebound_m: float = -0.003
    inward_stop_m: float = -0.012
    locked_stop_m: float = 0.001
    pop_target_m: float = 0.032
    outward_stop_m: float = 0.115
    stiffness_n_m: float = 90.0
    damping_n_s_m: float = 12.0
    free_damping_n_s_m: float = 0.6
    max_spring_force_n: float = 6.0


class PushLatch:
    """Measured-stroke hysteresis for one initially latched opening cycle."""

    def __init__(self, settings=None):
        self.settings = settings or PushLatchSettings()
        self.phase = "locked"
        self.compressed_s = 0.0

    def update(self, travel_m, dt_s):
        if not math.isfinite(travel_m) or not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("Latch requires finite travel and positive physics dt")
        p = self.settings
        if self.phase == "locked":
            self.compressed_s = self.compressed_s + dt_s if travel_m <= -p.compression_m else 0.0
            if self.compressed_s >= p.compression_hold_s - 1e-9:
                self.phase = "armed"
        elif self.phase == "armed" and travel_m >= p.rebound_m:
            self.phase = "open"
        # The ejector only pushes through its short stroke. It cannot drive
        # the drawer to full opening or pull it shut after the robot lets go.
        spring_active = travel_m < p.pop_target_m
        return {
            "phase": self.phase,
            "upper_limit_m": p.outward_stop_m if self.phase == "open" else p.locked_stop_m,
            "stiffness": p.stiffness_n_m if spring_active else 0.0,
            "damping": p.damping_n_s_m if spring_active else p.free_damping_n_s_m,
        }


def _tray(prefix, color):
    return (
        box(prefix + "_floor", (0.140, 0.124, 0.004), (0, 0, 0.002), color),
        box(prefix + "_left", (0.140, 0.004, 0.046), (0, -0.060, 0.023), color),
        box(prefix + "_right", (0.140, 0.004, 0.046), (0, 0.060, 0.023), color),
        box(prefix + "_back", (0.004, 0.124, 0.046), (-0.068, 0, 0.023), color),
        box(prefix + "_front", (0.008, 0.144, 0.055), (0.071, 0, 0.0275), color),
        box(prefix + "_tab", (0.018, 0.038, 0.020), (0.079, 0, 0.044), GREEN if prefix == "upper" else GREY),
    )


def build():
    p = PushLatchSettings()
    # Local +X is the drawer front. Face the left arm along world -X,
    # leaving workspace for pressing, post-pop grasp and the full rail stroke.
    x, y, upper_z, lower_z = 0.43, 0.28, 0.101, 0.018
    yaw = math.pi
    upper = Article("upper_drawer", _tray("upper", RED), pose(x, y, upper_z, yaw=yaw),
                    pose(x - p.outward_stop_m, y, upper_z, yaw=yaw), 0.14)
    lower = Article("lower_drawer", _tray("lower", BLUE), pose(x, y, lower_z, yaw=yaw),
                    pose(x, y, lower_z, yaw=yaw), 0.14)

    def cabinet_part(name, size, offset, color):
        # Same local frame as the drawers, including the asymmetric back wall.
        return box(name, size, (x - offset[0], y - offset[1], offset[2]), color,
                   (0.0, 0.0, yaw))

    fixtures = (
        cabinet_part("cabinet_base", (0.19, 0.174, 0.016), (-0.007, 0, 0.008), GREY),
        cabinet_part("cabinet_separator", (0.19, 0.174, 0.008), (-0.007, 0, 0.095), BLUE),
        cabinet_part("cabinet_roof", (0.19, 0.174, 0.008), (-0.007, 0, 0.166), BLUE),
        cabinet_part("cabinet_left", (0.19, 0.008, 0.162), (-0.007, -0.083, 0.089), BLUE),
        cabinet_part("cabinet_right", (0.19, 0.008, 0.162), (-0.007, 0.083, 0.089), BLUE),
        cabinet_part("cabinet_back", (0.008, 0.174, 0.162), (-0.106, 0, 0.089), BLUE),
    )
    joints = (
        Joint("upper_drawer_slide", "", "upper_drawer", (x, y, upper_z),
              {"transX": (-p.locked_stop_m, -p.inward_stop_m, p.stiffness_n_m, p.damping_n_s_m)}),
        Joint("lower_drawer_fixed", "", "lower_drawer", (x, y, lower_z), {}),
    )
    return BenchSpec(
        replace(design("L3-16", "上层抽屉按压解锁、弹出与拉开", (upper, lower), fixtures, friction=0.15),
                version="l3-16-household-v4-front"),
        joints=joints,
        mechanism="measured_stroke_push_latch_and_passive_ejector_proxy",
        limitations=("One opening cycle of an engineered push-to-open latch: measured compression/rebound "
                     "switches a rail limit; a bounded passive spring pushes through 32 mm only. "
                     "No commercial heart-cam geometry, automatic re-latching, fracture, or calibrated real forces. "
                     "Lower drawer is fixed closed. Cabinet is fixed; full opening stays rail-supported."),
        facts=("Only the UPPER red-front drawer is the target; the lower blue drawer remains closed. "
               "The fixed cabinet front faces the LEFT arm, world -X; "
               "do not reposition or rotate it. Approach its front from the -X side. "
               "Inward pressing is world +X; outward pop-out and pulling are -X, not along world Y. "
               "Press the upper front inward about 8-10 mm, below its green tab; never exceed the 12 mm inward stop. "
               "A full press arms the latch; release pressure and withdraw to allow rebound and a roughly "
               "20-40 mm passive pop-out. Pulling alone cannot unlock it. After observing pop-out, grasp the "
               "exposed green tab (38 mm wide), then pull along the rail. Maximum outward travel is 115 mm; "
               "at least 100 mm is required. Release and let it settle for one simulation second. "
               "Do not lift out the drawer or close it again. No force or lock-state sensor is supplied."),
        parameters={"press_m": p.compression_m, "pop_min_m": 0.018, "open_min_m": 0.100,
                    "outward_stop_m": p.outward_stop_m, "inward_stop_m": p.inward_stop_m,
                    "max_speed_m_s": 0.35, "max_finger_force_n": 40.0, "hold_s": 1.0,
                    "outward_axis_world": (-1.0, 0.0, 0.0), "latch": asdict(p)},
    )
