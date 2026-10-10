"""Public dimensions for a deliberately large teaching screw (metres)."""

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class ScrewSpec:
    version: str = "l3-05-short-half-turn-v2"
    core_radius: float = 0.013
    crest_radius: float = 0.018
    head_radius: float = 0.022
    shaft_length: float = 0.030
    head_height: float = 0.018
    pitch: float = 0.012
    thread_start: float = 0.008
    thread_turns: float = 1.5
    thread_root_width: float = 0.006
    thread_crest_width: float = 0.003
    radial_clearance: float = 0.0015
    segments_per_turn: int = 32
    socket_radius: float = 0.032
    socket_floor: float = 0.005
    socket_rim: float = 0.035
    socket_thread_start: float = 0.010
    socket_thread_turns: float = 1.5
    friction: float = 0.15
    required_depth: float = 0.015
    required_threaded_advance: float = 0.006
    required_turn_rad: float = math.pi
    alignment_tolerance: float = 0.0025
    tilt_tolerance_rad: float = math.radians(10)
    stability_s: float = 1.0
    contact_threshold: float = 0.05
    source_xy: tuple = (0.36, 0.40)
    socket_xy: tuple = (0.36, 0.23)
    table_z: float = 0.0

    @property
    def height(self):
        return self.shaft_length + self.head_height

    @property
    def initial_pose(self):
        # The pose origin is the flat shaft tip. The part stands independently.
        return (*self.source_xy, self.table_z + 0.001, 1, 0, 0, 0)


SPEC = ScrewSpec()


def task_spec(config):
    """Keep legacy recordings interpretable; new tasks use the delivered lab."""
    if config.raw.get("simulation", {}).get("environment") == "tuojing_lab_v4":
        return replace(
            SPEC,
            version="l3-05-derived-anchor-v3",
            table_z=0.78,
            source_xy=(-0.08, 1.30),
            socket_xy=(0.10, 1.30),
            stability_s=0.0,
        )
    return SPEC
