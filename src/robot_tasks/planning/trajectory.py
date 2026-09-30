"""Validate and retime planned joint paths before applying any drive target."""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class JointPath:
    positions: tuple[tuple[float, ...], ...]
    times_s: tuple[float, ...]

    @classmethod
    def from_waypoints(cls, positions, start, limits, *, max_step_rad=0.01):
        if not np.isfinite(max_step_rad) or max_step_rad <= 0:
            raise ValueError("Maximum joint increment must be finite and positive")
        q = np.asarray(positions, dtype=float)
        if (
            q.ndim != 2
            or q.shape[1] != len(limits)
            or not len(q)
            or not np.isfinite(q).all()
        ):
            raise ValueError("Planner returned an invalid joint trajectory")
        low = np.array([x.lower for x in limits])
        high = np.array([x.upper for x in limits])
        speed = np.array([x.velocity for x in limits])
        if np.any(q < low) or np.any(q > high) or np.any(speed <= 0):
            raise ValueError("Planner trajectory exceeds configured joint limits")
        start = np.asarray(start, dtype=float)
        if (
            start.shape != q.shape[1:]
            or not np.isfinite(start).all()
            or np.max(np.abs(q[0] - start)) > 0.02
        ):
            raise ValueError(
                "Planner path does not start at the measured configuration"
            )
        # Preserve its polyline; never interpolate straight from start to final IK.
        points = [start]
        times = [0.0]
        for target in q[1:]:
            delta = target - points[-1]
            count = max(1, int(np.ceil(np.max(np.abs(delta)) / max_step_rad)))
            duration = max(float(np.max(np.abs(delta) / speed)), 1e-4)
            origin, t0 = points[-1], times[-1]
            for i in range(1, count + 1):
                points.append(origin + delta * i / count)
                times.append(t0 + duration * i / count)
        return cls(tuple(tuple(float(x) for x in p) for p in points), tuple(times))

    @property
    def duration_s(self):
        return self.times_s[-1]

    def sample(self, time_s):
        q = np.asarray(self.positions)
        return tuple(
            float(np.interp(time_s, self.times_s, q[:, j])) for j in range(q.shape[1])
        )
