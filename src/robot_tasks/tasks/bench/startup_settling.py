"""Passive scene readiness, before model observations; never moves or freezes props."""

import numpy as np

from ...robots.transforms import pose_transform


class SupportedRest:
    """Require full-body rest and support continuously, including axial rotation."""

    def __init__(self, weight_n, duration_s=1.0):
        self.weight_n = weight_n
        self.duration_s = duration_s
        self.anchor = None

    def update(self, time_s, row):
        pose = pose_transform(row['pose'])
        quiet = (np.linalg.norm(row['linear_velocity']) < .001
                 and np.linalg.norm(row['angular_velocity']) < .015
                 and .8 * self.weight_n < row['bench_force_n'] < 1.2 * self.weight_n
                 and max(row['finger_forces'], default=0.) < .01)
        if not quiet:
            self.anchor = None
            return False
        if self.anchor is not None:
            start, previous = self.anchor
            rotation = previous[:3, :3].T @ pose[:3, :3]
            angle = np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))
            if np.linalg.norm(pose[:3, 3]-previous[:3, 3]) > .0005 or angle > .01:
                self.anchor = None
            elif time_s-start >= self.duration_s-1e-9:
                return True
        if self.anchor is None:
            self.anchor = (time_s, pose)
        return False


def settle_rose(scene, tick, *, max_wait_s=120.):
    """Only physics ticks and measurements; no pose/velocity writes or robot commands."""
    start = scene.backend.time_s()
    weight = scene.design.articles[0].mass * 9.81
    # A longer window covers the observed rocking cycle, not just a slow
    # turning point immediately before it rolls back in the other direction.
    monitor = SupportedRest(weight, duration_s=2.0)
    while scene.backend.time_s()-start < max_wait_s:
        tick(phase='task_initialization', score=False)
        row = scene.measure()['rose']
        now = scene.backend.time_s()
        if monitor.update(now, row):
            return {'protocol': 'passive_supported_rest_v1', 'ready': True,
                    'wait_s': now-start, 'stable_window_s': monitor.duration_s,
                    'linear_speed_m_s': float(np.linalg.norm(row['linear_velocity'])),
                    'angular_speed_rad_s': float(np.linalg.norm(row['angular_velocity'])),
                    'bench_force_n': row['bench_force_n'],
                    'pose_or_velocity_forced': False}
    raise RuntimeError('L1-09 rose did not reach supported rest before startup timeout; GPT was not called')
