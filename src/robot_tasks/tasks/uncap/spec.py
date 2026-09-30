"""Public geometry and scoring contract for the simplified L2-01 variant."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MarkerSpec:
    version: str = "l2-01-table-pickup-v3"
    body_radius: float = 0.015
    body_length: float = 0.120
    cap_radius: float = 0.021
    cap_length: float = 0.060
    overlap: float = 0.020
    sleeve_interference: float = 0.0002
    sleeve_stiffness: float = 5000.0
    sleeve_damping: float = 5.0
    sleeve_friction: float = 0.4
    contact_threshold: float = 0.05
    max_finger_force: float = (
        25.0  # Simulation protocol threshold, not a material damage law.
    )
    max_pair_force: float = 15.0
    stability_s: float = 1.0

    @property
    def cap_offset(self):
        return self.body_length / 2 + self.cap_length / 2 - self.overlap


SPEC = MarkerSpec()
TARGETS = {"body": (0.38, 0.28), "cap": (0.38, -0.28)}
TARGET_HALF_SIZE = 0.11


def surface(name):
    radius = SPEC.body_radius if name == "body" else SPEC.cap_radius
    length = SPEC.body_length if name == "body" else SPEC.cap_length
    return np.array(
        [
            (radius * math.cos(a), radius * math.sin(a), z)
            for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)
            for z in np.linspace(-length / 2, length / 2, 5)
        ]
    )
