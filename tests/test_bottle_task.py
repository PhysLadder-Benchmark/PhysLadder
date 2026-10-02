"""L3-03 dimensions, engaged rotation and supported final-state regressions."""

import math
import unittest
from dataclasses import replace

import numpy as np

from robot_tasks.integrationtest.bottle_clearance import box_vertices, separation
from robot_tasks.registry import resolve_task
from robot_tasks.robots.transforms import origin_transform
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.scoring_l3 import bottle
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.l1.specs import local_corners
from test_household_tasks import measurement


class BottleTaskTests(unittest.TestCase):
    def setUp(self):
        self.spec = build_spec('L3-03')
        self.rows = measurement(self.spec)
        self.evaluator = BenchEvaluator(self.spec, {a.name: a.initial for a in self.spec.design.articles})

    def test_grasp_regions_fit_gripper_and_clear_body_shoulder(self):
        body, cap = self.spec.design.articles
        shell = next(p for p in body.parts if p.name == 'bottle_body')
        _, config = resolve_task('L3-03')
        self.assertEqual(shell.shape, 'box')
        self.assertAlmostEqual(shell.size[0], shell.size[1])
        self.assertGreaterEqual(config.robot.gripper.width_limits_m[1] - shell.size[0], .019)
        cap_bounds = local_corners(cap)
        self.assertLess(np.ptp(cap_bounds[:, 0]), config.robot.gripper.width_limits_m[1] - .020)
        mouth = self.spec.parameters['initial_cap_z'] - self.spec.parameters['cap_half']
        self.assertGreaterEqual(mouth - shell.size[2], .029)
        self.assertGreater(self.spec.parameters['initial_cap_z'] - shell.size[2] / 2, .10)

    def test_thread_and_scorer_reference_same_seated_geometry(self):
        body, cap = self.spec.design.articles
        p = self.spec.parameters
        self.assertAlmostEqual(cap.initial[2] - body.initial[2], p['initial_cap_z'])
        self.assertAlmostEqual(p['rise'], p['neck_top'] + p['cap_half'] + .004 - p['initial_cap_z'])
        bounds = []
        for article in (body, cap):
            thread = replace(article, parts=tuple(x for x in article.parts if x.name.startswith('thread_')))
            z = local_corners(thread)[:, 2] + article.initial[2]
            bounds.append((z.min(), z.max()))
        self.assertGreater(min(b[1] for b in bounds) - max(b[0] for b in bounds), .006)
        _, metrics, _ = bottle(self.evaluator, self.rows)
        self.assertAlmostEqual(metrics['cap_rise_m'], 0)
        self.assertLess(metrics['axial_clearance_m'], 0)

    def turn(self, supported=True):
        rows, p = self.rows, self.spec.parameters
        rows['bottle']['fingers']['left'] = [1, 1]
        rows['cap']['fingers']['right'] = [1, 1]
        rows['bottle']['bench_force_n'] = 2 if supported else 0
        for i in range(21):
            angle = i * .4
            rows['cap']['pose'][3:] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
            rows['cap']['pose'][2] = rows['bottle']['pose'][2] + p['initial_cap_z'] + angle * p['pitch'] / (2 * math.pi)
            self.evaluator.update((i + 1) * .1, rows)

    def test_suspended_bottle_does_not_earn_supported_unscrewing(self):
        self.turn(supported=False)
        self.assertEqual(self.evaluator.net_turn, 0)
        self.assertNotIn('threaded_disengagement', self.evaluator.flags)

    def test_turn_alone_insufficient_then_detach_and_release_completes(self):
        self.turn()
        self.assertGreater(self.evaluator.net_turn, 2 * math.pi)
        self.assertNotIn('threaded_disengagement', self.evaluator.flags)
        p = self.spec.parameters
        self.rows['cap']['pose'][2] = self.rows['bottle']['pose'][2] + p['neck_top'] + p['cap_half'] + .006
        self.evaluator.update(2.2, self.rows)
        self.assertIn('threaded_disengagement', self.evaluator.flags)
        self.rows['bottle']['fingers']['left'] = [0, 0]
        self.rows['cap']['fingers']['right'] = [0, 0]
        self.rows['cap']['pose'][:3] = [.48, 0, p['cap_half']]
        self.rows['cap']['bench_force_n'] = .25
        for i in range(23, 38):
            result = self.evaluator.update(i * .1, self.rows)
        self.assertEqual(result.status, 'succeeded')
        self.rows['cap']['bench_force_n'] = 0
        self.assertEqual(self.evaluator.update(3.8, self.rows).status, 'running')

    def test_clearance_boxes_report_overlap_and_separation_under_rotation(self):
        a = box_vertices(np.eye(4), (-1, -1, -1), (1, 1, 1))
        b = box_vertices(origin_transform((3, 0, 0), (0, 0, .3)), (-1, -1, -1), (1, 1, 1))
        self.assertGreater(separation(a, b), .7)
        self.assertAlmostEqual(separation(a, b), separation(b, a))
        b = box_vertices(origin_transform((.2, 0, 0), (0, 0, .3)), (-1, -1, -1), (1, 1, 1))
        self.assertLess(separation(a, b), 0)


if __name__ == '__main__':
    unittest.main()
