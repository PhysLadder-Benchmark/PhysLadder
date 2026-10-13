"""Passive scene readiness, before model observations; never moves or freezes props."""

import numpy as np

from ...robots.transforms import pose_transform


class SupportedRest:
    """Require supported rest; a stem guide may permit harmless axial rotation."""

    def __init__(self, weight_n, duration_s=1.0, support_range=(.8,1.2), ignore_axial_spin=False,
                 position_tolerance_m=.0005, tilt_tolerance_rad=.01,
                 linear_speed_m_s=.001, angular_speed_rad_s=.015):
        self.weight_n = weight_n
        self.support_range = support_range
        self.ignore_axial_spin = ignore_axial_spin
        self.duration_s = duration_s
        self.position_tolerance_m = position_tolerance_m
        self.tilt_tolerance_rad = tilt_tolerance_rad
        self.linear_speed_m_s = linear_speed_m_s
        self.angular_speed_rad_s = angular_speed_rad_s
        self.anchor = None

    def update(self, time_s, row):
        pose = pose_transform(row['pose'])
        angular = np.asarray(row['angular_velocity'])
        if self.ignore_axial_spin:
            axis = pose[:3,2]
            angular = angular - axis * np.dot(angular, axis)
        quiet = (np.linalg.norm(row['linear_velocity']) < self.linear_speed_m_s
                 and np.linalg.norm(angular) < self.angular_speed_rad_s
                 and self.support_range[0] * self.weight_n < row['bench_force_n'] < self.support_range[1] * self.weight_n
                 and max(row['finger_forces'], default=0.) < .01)
        if not quiet:
            self.anchor = None
            return False
        if self.anchor is not None:
            start, previous = self.anchor
            rotation = previous[:3, :3].T @ pose[:3, :3]
            angle = np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))
            if self.ignore_axial_spin:
                angle = np.arccos(np.clip(previous[:3,2] @ pose[:3,2], -1, 1))
            if np.linalg.norm(pose[:3, 3]-previous[:3, 3]) > self.position_tolerance_m or angle > self.tilt_tolerance_rad:
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
    sources = scene.spec.parameters.get('source_support_articles', ())
    monitor = SupportedRest(weight, duration_s=2.0, support_range=(.5,3.) if sources else (.8,1.2),
                            ignore_axial_spin=scene.spec.parameters.get('source_ignore_axial_spin', bool(sources)),
                            **scene.spec.parameters.get('source_rest_tolerances', {}))
    while scene.backend.time_s()-start < max_wait_s:
        tick(phase='task_initialization', score=False)
        rows = scene.measure()
        row = rows['rose'].copy()
        if sources:
            # Only declared source props satisfy startup support. A flower
            # resting against a real charger may lean; do not require the old
            # inferred guide's near-vertical orientation.
            row['bench_force_n'] = sum(row['pair_forces'].get(source, 0.) for source in sources)
            if scene.spec.parameters.get('source_support_vertical'):
                # A charger can wedge the stem against the cup: large opposing
                # lateral normals are not excess vertical support. Keep signs
                # so a downward contact cannot count as weight support either.
                row['bench_force_n'] = sum(row['pair_force_vectors'][source][2] for source in sources)
            cup = pose_transform(rows['source_cup']['pose'])
            flower = pose_transform(row['pose'])
            local = np.linalg.inv(cup) @ flower
            minimum_up = np.cos(scene.spec.parameters['source_max_tilt_rad']) if 'source_max_tilt_rad' in scene.spec.parameters else .97
            if (cup[2,2] < .98 or flower[2,2] < minimum_up
                    or np.linalg.norm(local[:2,3]) > .035
                    or not -.001 < local[2,3] < .1075
                    or rows['rose']['bench_force_n'] > .01):
                monitor.anchor = None
                continue  # A toppled source arrangement is not a ready scene.
            if 'source_charger_pins_min_up' in scene.spec.parameters:
                charger = pose_transform(rows['source_charger']['pose'])
                if -charger[2,2] < scene.spec.parameters['source_charger_pins_min_up']:
                    monitor.anchor = None
                    continue  # Native pins point along -Z; USB-up is the wrong layout.
        now = scene.backend.time_s()
        if monitor.update(now, row):
            return {'protocol': 'passive_supported_rest_v1', 'ready': True,
                    'wait_s': now-start, 'stable_window_s': monitor.duration_s,
                    'linear_speed_m_s': float(np.linalg.norm(row['linear_velocity'])),
                    'angular_speed_rad_s': float(np.linalg.norm(row['angular_velocity'])),
                    'support_articles': sources or ('bench',),
                    'support_force_n': row['bench_force_n'],
                    'support_force_basis': ('signed_world_z' if scene.spec.parameters.get('source_support_vertical')
                                            else 'contact_magnitudes'),
                    'axial_spin_ignored': monitor.ignore_axial_spin,
                    'rest_tolerances': {'position_m': monitor.position_tolerance_m,
                                        'tilt_rad': monitor.tilt_tolerance_rad,
                                        'linear_speed_m_s': monitor.linear_speed_m_s,
                                        'angular_speed_rad_s': monitor.angular_speed_rad_s},
                    'pose_or_velocity_forced': False}
    raise RuntimeError('L1-09 flower did not reach supported rest before startup timeout; GPT was not called')
