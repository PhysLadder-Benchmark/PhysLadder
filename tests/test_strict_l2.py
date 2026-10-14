"""Adversarial CPU tests for strict manipulation; no model or simulator calls."""
import unittest
import numpy as np
from test_household_tasks import measurement
from robot_tasks.tasks.bench.l2_alignment import align_spec, PROFILE
from robot_tasks.tasks.bench.spec import build_spec
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.registry import resolve_task
from robot_tasks.paths import PROJECT_ROOT


def move(row, delta):
    row['pose'][:3]=(np.asarray(row['pose'][:3])+delta).tolist()
    row['corners']=(np.asarray(row['corners'])+delta).tolist()


class StrictL2Tests(unittest.TestCase):
    def setup_task(self,tid):
        self.spec=align_spec(build_spec(tid),{'real2sim_profile':PROFILE})
        self.rows=measurement(self.spec)
        self.e=BenchEvaluator(self.spec,{a.name:a.initial for a in self.spec.design.articles})
    def marker_rest(self):
        self.setup_task('L2-01')
        for r in self.rows.values():r['fixture_force_n']=.1
        self.e.update(.1,self.rows)
    def marker_lift(self):
        for r in self.rows.values():
            move(r,np.array([0,0,.08]));r['fixture_force_n']=0
        self.rows['body']['fingers']['left']=[1,1]
        self.e.update(.2,self.rows);self.e.update(.5,self.rows)
    def separate(self):
        from robot_tasks.robots.transforms import pose_transform
        axis=pose_transform(self.rows['body']['pose'])[:3,2]
        move(self.rows['cap'],axis*.05)
    def test_marker_requires_pickup_before_separation(self):
        self.marker_rest()
        self.rows['body']['fingers']['left']=[1,1]
        self.rows['cap']['fingers']['right']=[1,1]
        self.separate()
        self.assertEqual(self.e.update(.2,self.rows).status,'failed')
    def test_marker_cannot_skip_cap_grasp_before_pull(self):
        self.marker_rest();self.marker_lift()
        self.rows['cap']['fingers']['right']=[1,1];self.separate()
        self.assertEqual(self.e.update(.6,self.rows).status,'failed')
    def test_marker_ordered_sequence_and_final_hold(self):
        self.marker_rest();self.marker_lift()
        self.rows['cap']['fingers']['right']=[1,1];self.e.update(.6,self.rows)
        self.separate();self.assertEqual(self.e.update(.7,self.rows).status,'running')
        self.assertEqual(self.e.update(1.,self.rows).status,'succeeded')
        self.rows['cap']['fingers']['right']=[0,0]
        self.assertEqual(self.e.update(1.1,self.rows).status,'running')
    def test_marker_lost_barrel_grip_requires_new_stable_pickup(self):
        self.marker_rest();self.marker_lift()
        self.rows['body']['fingers']['left']=[0,0]
        self.e.update(.6,self.rows)
        self.rows['body']['fingers']['left']=[1,1]
        self.rows['cap']['fingers']['right']=[1,1]
        self.e.update(.7,self.rows);self.separate()
        self.assertEqual(self.e.update(.8,self.rows).status,'failed')
    def test_marker_dual_grip_metrics_are_json_serializable(self):
        import json
        from dataclasses import asdict
        self.marker_rest();self.marker_lift()
        self.rows['cap']['fingers']['right']=[1,1]
        json.dumps(asdict(self.e.update(.6,self.rows)),allow_nan=False)
    def test_moving_released_book_metrics_are_json_serializable(self):
        import json
        from dataclasses import asdict
        self.books_setup()
        self.rows['middle_book']['fingers']['right']=[0,0]
        self.rows['middle_book']['linear_velocity']=[.1,0,0]
        json.dumps(asdict(self.e.update(.2,self.rows)),allow_nan=False)
    def test_marker_premature_fall_is_failure(self):
        self.marker_rest();self.marker_lift();self.separate()
        self.assertEqual(self.e.update(.6,self.rows).status,'failed')
    def books_setup(self):
        self.setup_task('L2-03')
        self.rows['lower_book']['fixture_force_n']=1.
        self.rows['upper_book']['pair_forces']['lower_book']=1.
        self.rows['middle_book']['fingers']['right']=[1,1]
        self.e.update(.1,self.rows)
    def books_move(self,distance):
        move(self.rows['middle_book'],np.asarray(self.spec.parameters['outward_axis_world'])*distance)
    def test_twenty_mm_is_not_full_extraction(self):
        self.books_setup();self.books_move(.02)
        self.assertFalse(self.e.update(.2,self.rows).metrics['criteria']['stages']['fully_extracted_while_held'])
    def test_complete_pull_release_and_settle(self):
        self.books_setup();self.books_move(.104)
        self.e.update(.2,self.rows)
        r=self.rows['middle_book'];r['fingers']['right']=[0,0];r['bench_force_n']=1.
        self.assertEqual(self.e.update(.3,self.rows).status,'running')
        self.assertEqual(self.e.update(.9,self.rows).status,'succeeded')
    def test_drop_neighbour_then_restore_does_not_pass(self):
        self.books_setup();self.books_move(.104)
        self.rows['upper_book']['pair_forces']['lower_book']=0
        self.rows['upper_book']['bench_force_n']=1
        self.e.update(.2,self.rows)
        self.assertEqual(self.e.update(.5,self.rows).status,'failed')
    def test_geometric_clearance_without_actual_held_extraction_fails(self):
        self.books_setup();self.rows['middle_book']['fingers']['right']=[0,0]
        self.books_move(.104);self.rows['middle_book']['bench_force_n']=1
        self.e.update(.2,self.rows)
        self.assertEqual(self.e.update(1.,self.rows).status,'running')
    def test_contacting_books_do_not_count_as_fully_out(self):
        self.books_setup();self.books_move(.104)
        self.rows['middle_book']['pair_forces']['upper_book']=1.
        self.assertFalse(self.e.update(.2,self.rows).metrics['criteria']['stages']['fully_extracted_while_held'])
    def test_same_confirmed_box_dimensions(self):
        for tid in ('L2-01','L2-03','L2-06'):
            self.setup_task(tid);p=self.spec.parameters
            self.assertEqual(p['support_box_dimensions_m'],(.2,.2,.1))
            self.assertEqual(p['support_box_scale'],(.2/.19,1.,1.))
    def test_canonical_prompt_overrides_stale_input(self):
        from dataclasses import replace
        for tid in ('L2-03','L2-07'):
            item,cfg=resolve_task(tid,config_path=PROJECT_ROOT/f'configs/tasks/l2_{tid[-2:]}_real2sim.local.toml')
            cfg=replace(cfg,instruction='STALE WRONG DIRECTION 20 mm all blue')
            prepared=item.factory.prepare_config(cfg)
            self.assertNotIn('STALE',prepared.instruction)
            if tid=='L2-03':self.assertIn('Fully extract',prepared.instruction)
            else:self.assertIn('three blue, two green',prepared.instruction)
    def test_drawer_rotated_axis_matches_rail(self):
        self.setup_task('L2-07')
        p=self.spec.parameters
        delta=np.asarray(self.spec.design.articles[0].initial[:3])-np.asarray(p['closed_pose'][:3])
        self.assertTrue(np.allclose(delta/np.linalg.norm(delta),p['outward_axis_world']))
    def test_legacy_is_preserved(self):
        self.assertNotIn('strict_manipulation_protocol',build_spec('L2-03').parameters)
        self.assertIn('Complete extraction is not required',build_spec('L2-03').facts)

if __name__=='__main__':unittest.main()
