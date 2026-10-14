"""CPU scoring boundaries, delivered geometry and backward compatibility."""
import unittest
import numpy as np
from test_household_tasks import measurement
from robot_tasks.tasks.bench.l2_alignment import align_spec,PROFILE
from robot_tasks.tasks.bench.pearl_pouring import pouring,cup_contains
from robot_tasks.tasks.bench.evaluation import BenchEvaluator

class Real2SimTests(unittest.TestCase):
    def setUp(self):
        self.spec=align_spec(pouring(),{'real2sim_profile':PROFILE})
        self.rows=measurement(self.spec)
        self.e=BenchEvaluator(self.spec,{a.name:a.initial for a in self.spec.design.articles})
        self.rows['receiver_cup']['bench_force_n']=0
    def finish(self,count):
        for a in self.spec.design.articles[2:2+count]:
            self.rows[a.name]['pose']=list(a.goal)
            self.rows[a.name]['bench_force_n']=0
        self.e.flags.add('loaded_source_lifted')
        self.e.update(.1,self.rows)
        return self.e.update(.7,self.rows)
    def test_five_pass_four_fail(self):
        self.assertEqual(self.finish(4).status,'running')
        self.rows['load_4']['pose']=list(self.spec.design.articles[-1].goal)
        self.rows['load_4']['bench_force_n']=0
        self.e.update(.8,self.rows)
        self.assertEqual(self.e.update(1.5,self.rows).status,'succeeded')
    def test_all_received_does_not_require_extra_cup_release(self):
        self.rows['source_cup']['fingers']['right']=[1,1]
        self.rows['source_cup']['bench_force_n']=0
        self.assertEqual(self.finish(5).status,'succeeded')
    def test_whole_16mm_sphere_not_8mm_point_proxy(self):
        self.finish(5)
        from robot_tasks.tasks.bench.native_geometry import cup_cavity
        z=cup_cavity(self.spec.parameters['cup_asset'])[2]-.006
        self.assertTrue(cup_contains((0,0,z),radius=.004,cup_asset=self.spec.parameters['cup_asset']))
        self.assertFalse(cup_contains((0,0,z),radius=.008,cup_asset=self.spec.parameters['cup_asset']))
        rec=self.rows['receiver_cup']['pose']
        self.rows['load_4']['pose'][:3]=[rec[0],rec[1],rec[2]+z]
        self.assertEqual(self.e.update(.8,self.rows).metrics['criteria']['received'],4)
    def test_drawer_must_pull_then_return_without_50mm_requirement(self):
        from robot_tasks.tasks.bench.spec import build_spec
        spec=align_spec(build_spec('L2-07'),{'real2sim_profile':PROFILE})
        rows=measurement(spec);row=rows['drawer'];initial=np.array(row['pose'][:3]);axis=np.array(spec.parameters['outward_axis_world'])
        e=BenchEvaluator(spec,{a.name:a.initial for a in spec.design.articles})
        row['fingers']['right']=[1,1];e.update(.1,rows)
        row['pose'][:3]=(initial+.006*axis).tolist();e.update(.2,rows)
        row['fingers']['right']=[0,0]
        self.assertEqual(e.update(.3,rows).status,'running')
        row['pose'][:3]=initial.tolist()
        self.assertEqual(e.update(.4,rows).status,'succeeded')
    def test_exact_five_nonoverlap_and_native_physics(self):
        p=self.spec.parameters
        self.assertEqual((p['count'],p['required_received'],p['minimum_loaded_lift_count']),(5,5,5))
        self.assertNotIn('ball_friction',p)
        points=np.array([a.initial[:3] for a in self.spec.design.articles[2:]])
        d=np.linalg.norm(points[:,None]-points[None,:],axis=2);np.fill_diagonal(d,np.inf)
        self.assertGreater(d.min(),.016)
        source=np.array(self.spec.design.articles[0].initial[:3])
        self.assertTrue(all(cup_contains(q-source,radius=.008,tolerance=-.0003,cup_asset=p['cup_asset']) for q in points))
        self.assertEqual(pouring().parameters['count'],200)

if __name__=='__main__':unittest.main()
