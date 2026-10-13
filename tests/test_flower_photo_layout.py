"""Native glass containment, measured load paths and metal-pins-up readiness."""
import unittest
from types import SimpleNamespace
import numpy as np
from robot_tasks.tasks.bench.native_geometry import cavity_fit, cup_cavity, polyhedron
from robot_tasks.tasks.bench.scoring_l1 import flower_cup_support
from robot_tasks.tasks.bench.startup_settling import settle_rose
from robot_tasks.robots.transforms import pose_transform
from test_real_l1_protocol import case


class FlowerPhotoLayoutTests(unittest.TestCase):
    def test_source_has_no_initial_cavity_penetration_and_pins_are_up(self):
        _,s,rows,_,_=case('L1-09')
        a={a.name:a for a in s.design.articles}
        inv=np.linalg.inv(pose_transform(a['source_cup'].initial))
        for name in ('rose','source_charger'):
            transform=inv@pose_transform(a[name].initial)
            fit,clearance=cavity_fit(a[name].asset.split(':')[1],transform,.00005,receptacle='ripple_paper_cup_20261011')
            self.assertTrue(fit,(name,clearance))
        self.assertGreater(-pose_transform(a['source_charger'].initial)[2,2],np.cos(np.deg2rad(20)))
        self.assertLess(a['rose'].initial[0],a['source_charger'].initial[0])
        initial=[np.array(a[n].initial[:3]) for n in s.parameters['target_fillers']]
        for i,v in enumerate(initial):
            for w in initial[i+1:]:self.assertGreaterEqual(np.linalg.norm(v-w),.016)
        for n in s.parameters['target_fillers']:
            self.assertEqual(a[n].mass,.00536165146212658)
            vertices,_,_=polyhedron(a[n].asset.split(':')[1],0)
            self.assertTrue(np.allclose(np.linalg.norm(vertices,axis=1),.008))

    def test_indirect_support_requires_connected_contained_unheld_marble(self):
        _,s,r,_,_=case('L1-09');p=s.parameters;t=pose_transform(r['target_cup']['pose'])
        n,m=p['target_fillers'][:2]
        for row in r.values():row['pair_forces']={};row['bench_force_n']=0.
        r['rose']['pair_forces'][n]=.12
        # Two marbles transmit the load to the cup; no direct flower/cup touch.
        r[n]['pair_forces'][m]=.12;r[m]['pair_forces']['target_cup']=.12
        def force():return flower_cup_support(p,r,t,'ripple_paper_cup_20261011')[0]
        self.assertAlmostEqual(force(),.12)
        r[m]['pair_forces']={};self.assertEqual(force(),0.)
        r[m]['pair_forces']['target_cup']=.12
        r[n]['fingers']['left']=[.3,.3];self.assertEqual(force(),0.)
        r[n]['fingers']['left']=[0.,0.]
        r[n]['pose']=list(r[n]['pose']);r[n]['pose'][0]+=.2
        self.assertEqual(force(),0.)
        r['rose']['pair_forces']['target_cup']=.12;self.assertAlmostEqual(force(),.12)

    def test_usb_up_instead_of_pins_up_never_passes_startup(self):
        _,s,rows,_,_=case('L1-09');now=[0.]
        for row in rows.values():row.update(linear_velocity=[0.,0.,0.],angular_velocity=[0.,0.,0.],finger_forces=[0.,0.])
        rows['rose'].update(bench_force_n=0.,pair_forces={'source_cup':.11772})
        rows['rose']['pair_force_vectors']={'source_cup':[1.,0.,.11772], 'source_charger':[-1.,0.,0.]}
        scene=SimpleNamespace(spec=s,design=s.design,backend=SimpleNamespace(time_s=lambda:now[0]),measure=lambda:rows)
        def tick(**kwargs):now[0]+=.1
        self.assertTrue(settle_rose(scene,tick,max_wait_s=3.)['ready'])
        rows['rose']['pair_force_vectors']['source_cup'][2]=-.11772
        with self.assertRaisesRegex(RuntimeError,'GPT was not called'):settle_rose(scene,tick,max_wait_s=3.)
        rows['rose']['pair_force_vectors']['source_cup'][2]=.11772
        rows['source_charger']['pose']=list(rows['source_charger']['pose'])
        rows['source_charger']['pose'][3:]=[1.,0.,0.,0.]
        with self.assertRaisesRegex(RuntimeError,'GPT was not called'):settle_rose(scene,tick,max_wait_s=3.)

if __name__=='__main__':unittest.main()
