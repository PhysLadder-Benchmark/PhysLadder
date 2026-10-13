"""Do not begin model decisions while the native rose is still rolling."""
import unittest
from types import SimpleNamespace

import numpy as np

from robot_tasks.tasks.bench.startup_settling import SupportedRest, settle_rose
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.bench.native_geometry import shapes, transformed_poly
from robot_tasks.robots.transforms import pose_transform


def row():
    return dict(pose=[0.,0.,.8,1.,0.,0.,0.],linear_velocity=[0.,0.,0.],
                angular_velocity=[0.,0.,0.],bench_force_n=.07848,finger_forces=[0.,0.])


class RoseStartupTests(unittest.TestCase):
    def test_full_window_and_reject_axial_spin_and_missing_support(self):
        monitor=SupportedRest(.07848)
        r=row()
        self.assertFalse(monitor.update(0.,r))
        self.assertFalse(monitor.update(.9,r))
        self.assertTrue(monitor.update(1.,r))
        r['angular_velocity']=[0.,0.,.1]
        self.assertFalse(monitor.update(1.1,r))
        r['angular_velocity']=[0.,0.,0.]
        self.assertFalse(monitor.update(1.2,r))
        r['bench_force_n']=0.
        self.assertFalse(monitor.update(3.,r))
        r['bench_force_n']=.07848;r['finger_forces']=[.2,0.]
        self.assertFalse(monitor.update(4.,r))

    def test_small_reported_velocity_cannot_hide_pose_drift(self):
        monitor=SupportedRest(.07848);r=row()
        monitor.update(0.,r)
        r['pose'][0]=.0006
        self.assertFalse(monitor.update(1.,r))
        self.assertFalse(monitor.update(1.5,r))
        self.assertTrue(monitor.update(2.,r))

    def test_initial_tulip_is_in_source_cup_not_destination(self):
        from robot_tasks.tasks.bench.native_geometry import cavity_fit
        from robot_tasks.tasks.bench.evaluation import BenchEvaluator
        from robot_tasks.tasks.bench.scoring_l1 import flower
        from test_household_tasks import measurement
        spec=build_spec('L1-09');a,cup,guide=spec.design.articles
        relative=np.linalg.inv(pose_transform(cup.initial))@pose_transform(a.initial)
        self.assertGreater(relative[2,2],.95)
        self.assertTrue(cavity_fit('cream_tulip_20261011',relative,.001,
                                  receptacle='ripple_paper_cup_20261011')[0])
        rows=measurement(spec);rows['rose']['pair_forces']['source_cup']=.11772
        e=BenchEvaluator(spec,{a.name:a.initial for a in spec.design.articles})
        self.assertFalse(flower(e,rows)[0])

    def test_real_source_charger_accepts_supported_lean_but_rejects_toppled_flower(self):
        now=[0.]
        r=row();r.update(pose=[.01,0.,.805,np.cos(.15),0.,np.sin(.15),0.],
                         bench_force_n=0.,pair_forces={'source_cup':.08,'source_charger':.04})
        cup=row()
        scene=SimpleNamespace(backend=SimpleNamespace(time_s=lambda:now[0]),
                              design=SimpleNamespace(articles=[SimpleNamespace(mass=.012)]),
                              spec=SimpleNamespace(parameters={'source_support_articles':('source_cup','source_charger'),
                                                              'source_max_tilt_rad':np.pi/4}),
                              measure=lambda:{'rose':r,'source_cup':cup})
        def tick(**kwargs):now[0]+=.1
        self.assertTrue(settle_rose(scene,tick,max_wait_s=3.)['ready'])
        r['pose'][3:]=[np.cos(.6),0.,np.sin(.6),0.]
        with self.assertRaisesRegex(RuntimeError,'GPT was not called'):
            settle_rose(scene,tick,max_wait_s=3.)

    def test_startup_timeout_never_forces_pose_or_velocity(self):
        now=[0.];r=row();r['angular_velocity']=[1.,0.,0.]
        scene=SimpleNamespace(backend=SimpleNamespace(time_s=lambda:now[0]),
                              design=SimpleNamespace(articles=[SimpleNamespace(mass=.008)]),
                              spec=SimpleNamespace(parameters={}),
                              measure=lambda:{'rose':r})
        def tick(**kwargs):
            self.assertEqual(kwargs,dict(phase='task_initialization',score=False))
            now[0]+=.1
        with self.assertRaisesRegex(RuntimeError,'GPT was not called'):
            settle_rose(scene,tick,max_wait_s=2.)
        np.testing.assert_equal(r['angular_velocity'],[1.,0.,0.])



class GuidedFlowerRestTests(unittest.TestCase):
    def test_guided_stem_allows_axial_spin_but_not_tilt_or_translation(self):
        monitor=SupportedRest(.11772,ignore_axial_spin=True)
        r=row();r['bench_force_n']=.11772;r['angular_velocity']=[0.,0.,2.]
        self.assertFalse(monitor.update(0.,r))
        self.assertTrue(monitor.update(1.,r))
        r['angular_velocity']=[.1,0.,2.]
        self.assertFalse(monitor.update(1.1,r))
        r['angular_velocity']=[0.,0.,2.]
        monitor.update(2.,r)
        r['pose'][0]+=.001
        self.assertFalse(monitor.update(3.,r))

if __name__=='__main__':unittest.main()
