"""Real-summary replay and measured completion, independent of any GPT success claim."""
import json
from dataclasses import replace
from pathlib import Path
import unittest
import numpy as np

from robot_tasks.registry import resolve_task
from robot_tasks.tasks.bench.arm_selection import resolve_bench_spec
from robot_tasks.tasks.bench.evaluation import BenchEvaluator
from robot_tasks.tasks.bench.real_l1 import PROTOCOL, TASKS
from robot_tasks.robots.transforms import pose_transform
from robot_tasks.tasks.l1.specs import local_corners
from robot_tasks.types import ArmState, Action, ArmCommand
from robot_tasks.runner import validate_action
from test_household_tasks import measurement

ROOT = Path(__file__).resolve().parents[1]


def case(task):
    _, config = resolve_task(task)
    spec = resolve_bench_spec(config.raw['task'], config.robot)
    rows = measurement(spec)
    robot = {m.name: ArmState(tuple(m.home_positions_rad), .07, (0.,)*6,
             tcp_pose_world_wxyz=(0.,1.6,1.5,1.,0.,0.,0.)) for m in config.robot.mounts}
    return config, spec, rows, robot, BenchEvaluator(spec, {a.name:a.initial for a in spec.design.articles})


def set_pose(spec, rows, name, pose):
    a = next(a for a in spec.design.articles if a.name == name)
    t = pose_transform(pose)
    rows[name]['pose'] = list(pose)
    rows[name]['corners'] = (local_corners(a)@t[:3,:3].T+t[:3,3]).tolist()


def pickup(spec, rows, robot, evaluator, name, arm, start=.1):
    pose = list(rows[name]['pose']); pose[2] += .04
    set_pose(spec, rows, name, pose)
    rows[name]['bench_force_n'] = 0.
    rows[name]['fingers'][arm] = [.3,.3]
    evaluator.update(start, rows, robot=robot)
    evaluator.update(start+.2, rows, robot=robot)
    rows[name]['fingers'][arm] = [0.,0.]


def at_goal(spec, rows, name):
    a = next(a for a in spec.design.articles if a.name == name)
    set_pose(spec, rows, name, a.goal)
    rows[name]['fixture_force_n'] = a.mass*9.81
    rows[name]['bench_force_n'] = 0.


class RealPromptTests(unittest.TestCase):
    def test_all_five_runtime_instructions_equal_source_summaries_exactly(self):
        source = json.loads((ROOT/'configs/prompts/l1_real_20261012.json').read_text())
        for task in TASKS:
            binding, config = resolve_task(task)
            self.assertEqual(binding.factory.prepare_config(config).instruction, source['tasks'][task]['instruction'])
            expected = 'feishu_real_l1_08_scene_v4_real_arm_names' if task == 'L1-08' else PROTOCOL
            self.assertEqual(resolve_bench_spec(config.raw['task'],config.robot).parameters['scoring_protocol'],expected)
        self.assertNotIn('L1-05',source['tasks'])
        self.assertEqual(resolve_task('L1-05')[1].raw['task'].get('arm_selection','fixed'),'fixed')

    def test_effective_arm_roles_match_real_trials(self):
        for task in TASKS:
            config,spec,_,_,_=case(task)
            self.assertEqual(config.arms,('left','right') if task=='L1-08' else ('left',))
            self.assertEqual(config.observation_arms,('right',) if task != 'L1-08' else ())
            self.assertEqual(spec.observers,config.observation_arms)

    def test_marker_entry_while_held_then_release_without_home_does_not_succeed(self):
        _,s,r,robot,e=case('L1-01');pickup(s,r,robot,e,'marker','left')
        at_goal(s,r,'marker');r['marker']['fingers']['left']=[.3,.3]
        self.assertEqual(e.update(.4,r,robot=robot).status,'running')
        r['marker']['fingers']['left']=[0.,0.]
        robot['left']=replace(robot['left'],joint_positions_rad=(.3,)*6)
        e.update(.5,r,robot=robot)
        self.assertEqual(e.update(1.6,r,robot=robot).status,'running')
        robot['left']=replace(robot['left'],joint_positions_rad=s.parameters['home_positions']['left'])
        self.assertEqual(e.update(1.7,r,robot=robot).status,'succeeded')

    def test_no_robot_evidence_or_no_pickup_cannot_pass(self):
        _,s,r,robot,e=case('L1-01');at_goal(s,r,'marker')
        e.update(.1,r,robot=robot)
        self.assertEqual(e.update(2.,r,robot=robot).status,'running')
        e.flags.add('real_handled_marker')
        self.assertEqual(e.update(3.,r).status,'running')

    def test_lost_support_or_drift_resets_stability(self):
        _,s,r,robot,e=case('L1-03');pickup(s,r,robot,e,'plug','left');at_goal(s,r,'plug')
        e.update(.4,r,robot=robot)
        r['plug']['fixture_force_n']=0.
        self.assertEqual(e.update(1.,r,robot=robot).status,'running')
        r['plug']['fixture_force_n']=.54
        self.assertEqual(e.update(1.1,r,robot=robot).status,'running')
        self.assertEqual(e.update(2.2,r,robot=robot).status,'succeeded')

    def test_plug_wrong_slot_and_still_held_rejected(self):
        _,s,r,robot,e=case('L1-03');pickup(s,r,robot,e,'plug','left');at_goal(s,r,'plug')
        r['plug']['fingers']['left']=[.3,.3]
        e.update(.4,r,robot=robot)
        self.assertEqual(e.update(2.,r,robot=robot).status,'running')
        r['plug']['fingers']['left']=[0.,0.]
        pose=list(r['plug']['pose']);pose[0]+=.064;set_pose(s,r,'plug',pose)
        e.update(2.1,r,robot=robot)
        self.assertEqual(e.update(3.2,r,robot=robot).status,'running')
        self.assertEqual(s.parameters['target_socket_index'],1)

    def test_cup_sequence_requires_left_placement_return_then_right_nesting_return(self):
        _,s,r,robot,e=case('L1-08')
        self.assertGreater(r['lower_cup']['pose'][0],r['upper_cup']['pose'][0])
        pickup(s,r,robot,e,'lower_cup','left')
        a=next(a for a in s.design.articles if a.name=='lower_cup');set_pose(s,r,'lower_cup',a.goal)
        r['lower_cup']['bench_force_n']=.14
        robot['left']=replace(robot['left'],joint_positions_rad=(.3,)*6)
        e.update(.4,r,robot=robot);e.update(1.5,r,robot=robot)
        self.assertNotIn('first_cup_placed_and_left_home',e.flags)
        robot['left']=replace(robot['left'],joint_positions_rad=s.parameters['home_positions']['left'])
        e.update(1.6,r,robot=robot)
        self.assertIn('first_cup_placed_and_left_home',e.flags)
        pickup(s,r,robot,e,'upper_cup','right',1.7)
        at_goal(s,r,'upper_cup');r['upper_cup']['pair_forces']['lower_cup']=.14;r['lower_cup']['bench_force_n']=.28
        robot['right']=replace(robot['right'],joint_positions_rad=(.3,)*6)
        e.update(2.,r,robot=robot)
        result=e.update(3.1,r,robot=robot);self.assertEqual(result.status,'running',result.metrics['criteria'])
        robot['right']=replace(robot['right'],joint_positions_rad=s.parameters['home_positions']['right'])
        self.assertEqual(e.update(3.2,r,robot=robot).status,'succeeded')

    def test_right_preparatory_motion_does_not_terminate_or_skip_first_stage(self):
        _,s,r,robot,e=case('L1-08');robot['right']=replace(robot['right'],joint_velocities_rad_s=(.2,)*6)
        e.update(.1,r,robot=robot)
        result=e.update(.3,r,robot=robot)
        self.assertEqual(result.status,'running')
        self.assertTrue(result.metrics['criteria']['process_warnings'])
        self.assertNotIn('first_cup_placed_and_left_home',e.flags)
        self.assertNotIn('real_process_violated',e.flags)

    def test_original_position_rejected_broad_handoff_passes_without_minimum_lift(self):
        _,s,r,robot,e=case('L1-08')
        # The scene description adds central transfer, but no minimum lift height.
        r['lower_cup']['fingers']['left']=[.3,.3]
        e.update(.1,r,robot=robot);e.update(.3,r,robot=robot)
        r['lower_cup']['fingers']['left']=[0.,0.]
        r['lower_cup']['bench_force_n']=.14
        e.update(.4,r,robot=robot);e.update(1.5,r,robot=robot)
        self.assertNotIn('first_cup_placed_and_left_home',e.flags)
        # Outside the former 6-cm target circle, inside a broad middle region.
        base=next(a for a in s.design.articles if a.name=='lower_cup')
        pose=list(base.goal);pose[0]+=.065;set_pose(s,r,'lower_cup',pose)
        e.update(1.6,r,robot=robot);e.update(2.7,r,robot=robot)
        self.assertIn('first_cup_placed_and_left_home',e.flags)
        r['upper_cup']['fingers']['right']=[.3,.3]
        e.update(2.8,r,robot=robot);e.update(3.,r,robot=robot)
        at_goal(s,r,'upper_cup');top=list(r['upper_cup']['pose']);top[0]+=.065
        set_pose(s,r,'upper_cup',top);r['upper_cup']['pair_forces']['lower_cup']=.14
        r['lower_cup']['bench_force_n']=.28
        # Holding a correctly nested cup still does not count as success.
        e.update(3.1,r,robot=robot)
        self.assertEqual(e.update(4.2,r,robot=robot).status,'running')
        r['upper_cup']['fingers']['right']=[0.,0.]
        e.update(4.3,r,robot=robot)
        self.assertEqual(e.update(5.4,r,robot=robot).status,'succeeded')

    def test_cup_stability_allows_small_common_drift_but_requires_support(self):
        from robot_tasks.tasks.bench.scoring_l1 import stable_outcome
        _,s,r,robot,e=case('L1-08')
        self.assertTrue(stable_outcome(e,'probe',[r['lower_cup']],True))
        pose=list(r['lower_cup']['pose']);pose[0]+=.003
        set_pose(s,r,'lower_cup',pose)
        self.assertTrue(stable_outcome(e,'probe',[r['lower_cup']],True))
        self.assertFalse(stable_outcome(e,'probe',[r['lower_cup']],False))

    def test_nesting_without_the_requested_arm_sequence_does_not_pass(self):
        _,s,r,robot,e=case('L1-08')
        at_goal(s,r,'lower_cup');at_goal(s,r,'upper_cup')
        r['upper_cup']['pair_forces']['lower_cup']=.14;r['lower_cup']['bench_force_n']=.28
        e.update(.1,r,robot=robot)
        self.assertEqual(e.update(2.,r,robot=robot).status,'running')

    def test_simultaneous_cup_commands_are_rejected_before_execution(self):
        c,_,_,_,_=case('L1-08')
        with self.assertRaisesRegex(ValueError,'only one arm'):
            validate_action(Action(commands=(ArmCommand('left','gripper',(.06,)),ArmCommand('right','gripper',(.06,))),duration_s=c.control_dt_s),c)

    def test_hat_needs_release_and_support_but_no_extra_dwell_or_home(self):
        _,s,r,robot,e=case('L1-07');pickup(s,r,robot,e,'hat','left');at_goal(s,r,'hat')
        r['hat']['target_hook_vertical_force_n']=.64
        robot['left']=replace(robot['left'],joint_positions_rad=(.3,)*6)
        r['hat']['fingers']['left']=[.3,.3]
        self.assertEqual(e.update(.4,r,robot=robot).status,'running')
        r['hat']['fingers']['left']=[0.,0.]
        r['hat']['angular_velocity']=[.2,0.,0.]
        self.assertEqual(e.update(.5,r,robot=robot).status,'succeeded')

    def test_flower_release_without_both_arms_home_does_not_succeed(self):
        _,s,r,robot,e=case('L1-09');pickup(s,r,robot,e,'rose','left');at_goal(s,r,'rose')
        r['rose']['pair_forces']['target_cup']=.12
        r['target_cup']['bench_force_n']=.19
        robot['right']=replace(robot['right'],joint_positions_rad=(.3,)*6)
        e.update(.4,r,robot=robot)
        self.assertEqual(e.update(1.5,r,robot=robot).status,'running')
        robot['right']=replace(robot['right'],joint_positions_rad=s.parameters['home_positions']['right'])
        self.assertEqual(e.update(1.6,r,robot=robot).status,'succeeded')

    def test_runtime_and_frozen_config_include_scene_context_without_changing_prompt(self):
        import tempfile
        from robot_tasks.batch import freeze_config
        from robot_tasks.config import load_config
        from robot_tasks.policies.operating_notes import operating_notes
        source=json.loads((ROOT/'configs/prompts/l1_real_20261012.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            for task in TASKS:
                binding,c=resolve_task(task)
                path=Path(temp)/(task+'.toml')
                freeze_config(task,path,ROOT/'configs/models/gpt.toml',ROOT/'configs/workflows/gpt_default.toml',
                              dict(reasoning_effort='xhigh',seed=0,wall_time_s=3600,max_calls=100))
                frozen=binding.factory.prepare_config(load_config(path))
                self.assertEqual(frozen.instruction,source['tasks'][task]['instruction'])
                self.assertIn(source['tasks'][task]['scene_context'],operating_notes(frozen)['startup_preparation'])

    def test_flower_source_and_movable_target_match_latest_scene(self):
        _,s,r,robot,e=case('L1-09')
        self.assertEqual(s.design.fixture_asset,'')
        self.assertNotIn('source_stem_guide',r)
        self.assertGreater(r['source_cup']['pose'][0],r['target_cup']['pose'][0])
        self.assertGreater(r['rose']['pose'][0],r['target_cup']['pose'][0])
        self.assertLess(abs(r['source_charger']['pose'][0]-r['source_cup']['pose'][0]),.03)
        self.assertIn('marble dimensions/mass estimated',s.parameters['alignment_gaps'][0])
        self.assertEqual(len(s.parameters['target_fillers']),10)
        pickup(s,r,robot,e,'rose','left')
        # Insertion into the original source must never count as target success.
        r['rose']['pair_forces']['source_cup']=.05
        e.update(.4,r,robot=robot)
        self.assertEqual(e.update(1.5,r,robot=robot).status,'running')
        at_goal(s,r,'rose')
        r['rose']['pair_forces']['target_cup']=.12
        r['target_cup']['bench_force_n']=.19
        # Move target and flower together: judge the measured cup, not old fixture pose.
        for name in ('target_cup','rose'):
            pose=list(r[name]['pose']);pose[0]+=.08;set_pose(s,r,name,pose)
        e.update(1.6,r,robot=robot)
        self.assertEqual(e.update(2.7,r,robot=robot).status,'succeeded')
        r['target_cup']['bench_force_n']=0.
        self.assertEqual(e.update(2.8,r,robot=robot).status,'running')
