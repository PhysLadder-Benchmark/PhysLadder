"""Proprioceptive settling only: a stalled finger is NOT a grasp sensor."""

from dataclasses import dataclass


@dataclass
class PositionStability:
    tolerance: float = 0.00025
    anchor: tuple = ()
    quiet_s: float = 0.0

    def update(self, positions, dt):
        positions = tuple(positions)
        if not self.anchor or max(abs(q-a) for q,a in zip(positions, self.anchor)) > self.tolerance:
            self.anchor, self.quiet_s = positions, 0.0
        else:
            self.quiet_s += dt
        return self.quiet_s >= .5


@dataclass
class GripperProgress:
    target: tuple = ()
    anchor: tuple = ()
    quiet_s: float = 0.0
    status: str = "moving"

    def update(self, positions, velocities, target, dt):
        positions, target = tuple(positions), tuple(target)
        if target != self.target:
            self.target, self.anchor, self.quiet_s = target, positions, 0.0
        errors = tuple(q - t for q, t in zip(positions, target))
        quiet = all(abs(v) < 0.003 for v in velocities)
        # Contact solvers can report oscillating instantaneous velocities even
        # with an essentially fixed opening. Require sustained measured position
        # stability rather than treating a single velocity sample as a stall.
        if max(abs(q - a) for q, a in zip(positions, self.anchor)) > 0.00025:
            self.anchor, self.quiet_s = positions, 0.0
        else:
            self.quiet_s += dt
        if max(abs(e) for e in errors) < 0.0015 and quiet:
            self.status = "at_target"
        elif self.quiet_s >= 0.5:
            if min(errors) >= -0.0015 and max(errors) >= 0.0015:
                self.status = "closing_stalled"
            elif max(errors) <= 0.0015 and min(errors) <= -0.0015:
                self.status = "opening_stalled"
            else:
                self.status = "stalled"
        else:
            self.status = "moving"
        return self.status
