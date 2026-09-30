"""Public dimensions for a deliberately large teaching screw (metres)."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ScrewSpec:
    version: str = "l3-05-coarse-thread-v1"
    core_radius: float = 0.013
    crest_radius: float = 0.018
    head_radius: float = 0.022
    shaft_length: float = 0.042
    head_height: float = 0.018
    pitch: float = 0.012
    thread_start: float = 0.008
    thread_turns: float = 2.0
    thread_root_width: float = 0.006
    thread_crest_width: float = 0.003
    radial_clearance: float = 0.0015
    segments_per_turn: int = 32
    socket_radius: float = 0.032
    socket_floor: float = 0.005
    socket_rim: float = 0.047
    socket_thread_start: float = 0.010
    socket_thread_turns: float = 2.5
    friction: float = 0.15
    required_depth: float = 0.021
    required_threaded_advance: float = 0.009
    required_turn_rad: float = 1.5 * math.pi
    alignment_tolerance: float = 0.0025
    tilt_tolerance_rad: float = math.radians(10)
    stability_s: float = 1.0
    contact_threshold: float = 0.05
    source_xy: tuple = (0.36, 0.40)
    socket_xy: tuple = (0.36, 0.23)

    @property
    def height(self):
        return self.shaft_length + self.head_height

    @property
    def initial_pose(self):
        # The pose origin is the flat shaft tip. The part stands independently.
        return (*self.source_xy, 0.001, 1, 0, 0, 0)


SPEC = ScrewSpec()
