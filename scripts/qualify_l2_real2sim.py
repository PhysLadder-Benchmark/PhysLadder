"""Deterministic, simulation-only L2 qualification. Never instantiates GPT or hardware."""
import argparse,json,time,re
from dataclasses import replace,asdict
from pathlib import Path
import numpy as np
from robot_tasks.registry import resolve_task
from robot_tasks.paths import PROJECT_ROOT
from robot_tasks.backends.isaacsim import IsaacSimBackend
from robot_tasks.robots.adapters import R5Adapter
from robot_tasks.runner import run_episode
from robot_tasks.types import Action,ArmCommand,InvalidSimulationState
from robot_tasks.robots.transforms import origin_transform,transform_pose,pose_transform
from robot_tasks.robots.preparation import preparation_action

p=argparse.ArgumentParser();p.add_argument('task',choices=['L2-06','L2-01','L2-03']);p.add_argument('--output',type=Path,required=True);p.add_argument('--orange-widths',default='0.05,0.045,0.042,0.04');a=p.parse_args()
spec,cfg=resolve_task(a.task,config_path=PROJECT_ROOT/f'configs/tasks/l2_{a.task[-2:]}_real2sim.local.toml')
cfg=spec.factory.prepare_config(cfg)
camera_source=PROJECT_ROOT/'configs/cameras/r5_tuojing_lab.toml'
text=camera_source.read_text()
text=re.sub(r'^resolution = \[(\d+), (\d+)\]',lambda m:f'resolution = [{int(m[1])//2}, {int(m[2])//2}]',text,flags=re.M)
text=re.sub(r'^intrinsics = \[(.*?)\]',lambda m:'intrinsics = '+json.dumps([float(x)/2 for x in m[1].split(',')]),text,flags=re.M)
a.output.parent.mkdir(parents=True,exist_ok=True)
camera_local=(a.output.parent/(a.output.name+'-cameras.toml')).resolve();camera_local.write_text(text)
raw={**cfg.raw,'simulation':{**cfg.raw['simulation'],'camera_resolution':[640,480],'camera_intrinsics':[580.,580.,320.,240.],'capture_render_passes':1,'camera_profile':str(camera_local),'fps':6},'execution':{**cfg.raw['execution'],'max_decisions':{'L2-06':9+2*len(a.orange_widths.split(',')),'L2-01':13,'L2-03':19}[a.task],'max_action_steps':45,'wait_steps':6,'settle_steps':6},'recording':{**cfg.raw['recording'],'save_rgb':True,'save_video':True}}
raw['runtime']={**raw['runtime'],'control_dt_s':1/6}
cfg=replace(cfg,raw=raw,control_dt_s=1/6,max_wall_time_s=1200)
task=spec.factory();a.output.mkdir(parents=True,exist_ok=True)
class Sequence:
    run_metadata={'policy':'deterministic_qualification_v1','gpt_calls':0,'real_hardware':False,
                  'privileged_geometry':True,'orange_widths_m':[float(v) for v in a.orange_widths.split(',')],'scope':'engineering test; not model benchmark or real2sim calibration'}
    def reset(self,seed):
        self.i=0;self.steps=None;self.book_delta=np.zeros(3);self.book_corrections=0
        import signal
        signal.signal(signal.SIGINT,signal.default_int_handler)
        (a.output/'qualification-driver.py').write_text(Path(__file__).read_text())
    def act(self,context):
        rows=task.scene.measure()
        with (a.output/'qualification-samples.jsonl').open('a') as f:
            f.write(json.dumps({'step':context.step,'simulation_time_s':context.observation.time_s,
                'robot':{n:asdict(s) for n,s in context.observation.robot.items()},
                'feedback':asdict(context.observation.feedback),'measurements':rows},default=lambda o:o.tolist() if hasattr(o,'tolist') else str(o))+'\n')
        if context.observation.feedback.status in ('rejected','collision_blocked','action_timeout','error'):
            raise RuntimeError('Qualification stopped on execution feedback: '+context.observation.feedback.message)
        if a.task=='L2-06' and (rows['orange']['minimum_volume_ratio']<=0 or rows['orange']['strain_p95']>.6):
            raise InvalidSimulationState('Qualification stopped: FEM inversion or strain >0.6; this is numerical validity, not tissue damage')
        if self.steps is None:self.steps=self.build(rows)
        if self.i>=len(self.steps):
            (a.output/'sequence-completed.json').write_text(json.dumps({'sequence_completed':True,'gpt_calls':0,'task_success':False,'note':'Sequence completion is not task success or material calibration'}))
            return Action((),cfg.control_dt_s,note='Post-sequence observation')
        label,commands=self.steps[self.i]
        if a.task=='L2-03' and label=='right_close_book':
            measured=np.asarray(context.observation.robot['right'].tcp_pose_world_wxyz[:3])
            error=self.book_contact-measured
            if np.linalg.norm(error)>.0015:
                if self.book_corrections>=2 or np.linalg.norm(error)>.008:
                    raise RuntimeError('Qualification stopped: actual TCP did not reach book contact center within 1.5 mm')
                self.book_delta+=error;self.book_corrections+=1
                print('QUALIFICATION_PHASE feedback_correct_book_TCP '+str(error.tolist()),flush=True)
                return Action((ArmCommand('right','cartesian_pose',tuple(self.book_contact+self.book_delta)+self.book_quat),),cfg.control_dt_s,note='Feedback correction using measured TCP; same book pose, collision checks retained')
        if a.task=='L2-03' and label=='pull_60mm':
            from robot_tasks.tasks.bench.evaluation import held
            if not held(rows['middle_book'],'right'):
                raise RuntimeError('Qualification stopped: middle book two-finger grip not verified; no extraction commanded')
        if a.task=='L2-03' and label=='settle_neighbours' and 'strict_middle_fully_extracted_held' not in task.evaluator.flags:
            raise RuntimeError('Qualification stopped: commanded pull did not produce verified complete extraction')
        if a.task=='L2-03':
            commands=tuple(replace(c,values=tuple(np.asarray(c.values[:3])+self.book_delta)+tuple(c.values[3:])) if c.arm=='right' and c.kind=='cartesian_pose' else c for c in commands)
        if a.task=='L2-01' and label=='lift_capped_marker':
            from robot_tasks.tasks.bench.evaluation import held
            if not held(rows['body'],'left'):
                raise RuntimeError('Qualification stopped: barrel two-finger contact not verified after closure')
        if a.task=='L2-01' and label=='right_pregrasp_cap' and 'strict_capped_lift_stable' not in task.evaluator.flags:
            raise RuntimeError('Qualification stopped: capped pickup and stable hold not verified; do not continue to cap approach')
        self.i+=1
        print('QUALIFICATION_PHASE '+label,flush=True)
        return Action(tuple(commands),cfg.control_dt_s,note=label)
    def build(self,rows):
        seq=[]
        def grip(arm,w):return ArmCommand(arm,'gripper',(w,))
        def tcp(arm,xyz,yaw=0):
            return ArmCommand(arm,'cartesian_pose',transform_pose(origin_transform(xyz,(0,np.pi/2,yaw))))
        def add(label,*cmds):seq.append((label,cmds))
        if a.task=='L2-06':
            center=np.asarray(rows['orange']['centroid']);top=rows['orange']['bounds_max'][2]
            base=next(m for m in cfg.robot.mounts if m.name=='right').base_pose_wxyz
            yaw=float(np.arctan2(center[1]-base[1],center[0]-base[0]))
            def tcp(arm,xyz):
                return ArmCommand(arm,'cartesian_pose',transform_pose(origin_transform(xyz,(0,np.pi/4,yaw))))
            add('open_to_70mm',grip('right',.07))
            add('approach_80mm_above_tilt45',tcp('right',(center[0],center[1],top+.08)))
            add('approach_30mm_above_tilt45',tcp('right',(center[0],center[1],top+.03)))
            add('contact_height_fixed_wrist',tcp('right',(center[0],center[1],top+.008)))
            for width in tuple(float(v) for v in a.orange_widths.split(',')):
                add(f'close_to_{width*1000:.0f}mm',grip('right',width));add('hold_one_second')
            add('release_70mm',grip('right',.07));add('recovery_observation');add('recovery_observation')
            add('withdraw',tcp('right',(center[0],center[1],top+.08)))
        elif a.task=='L2-01':
            def tcp(arm,xyz,yaw=0):
                angle=0. if arm=='left' else np.pi
                return ArmCommand(arm,'cartesian_pose',transform_pose(origin_transform(xyz,(0,np.pi/6,angle))))
            t=pose_transform(rows['body']['pose']);axis=t[:3,2]
            b=(t@np.array([0,0,.055,1]))[:3];c=(t@np.array([0,0,.137,1]))[:3]
            add('open_both',grip('left',.07),grip('right',.07))
            add('left_pregrasp',tcp('left',b+[0,0,.06]));add('left_grasp_height',tcp('left',b+[0,0,.016]))
            add('left_close_barrel',grip('left',.010));add('lift_capped_marker',tcp('left',b+[0,0,.056]));add('stable_capped_hold')
            add('right_pregrasp_cap',tcp('right',c+[0,0,.11]));add('right_cap_height',tcp('right',c+[0,0,.056]))
            add('right_close_cap',grip('right',.016));add('verify_two_hands_capped')
            add('pull_cap_axially',tcp('right',c+[0,0,.056]+axis*.05));add('hold_separated');add('hold_separated')
        else:
            # Wrist rotated so jaw closing direction spans book thickness.
            m=np.mean(rows['middle_book']['corners'],axis=0);u=np.mean(rows['upper_book']['corners'],axis=0)
            def tcp(arm,xyz,yaw=0):
                base=next(b for b in cfg.robot.mounts if b.name==arm).base_pose_wxyz
                angle=float(np.arctan2(u[1]-base[1],u[0]-base[0]))
                return ArmCommand(arm,'cartesian_pose',transform_pose(origin_transform(xyz,(0,np.pi/4,angle))))
            axis=np.asarray(task.spec.parameters['outward_axis_world']);edge=max(np.asarray(rows['middle_book']['corners'])@axis)
            contact=m+axis*(edge-m@axis-.012)
            # Side approach along the extraction axis; jaw closing axis is Z.
            approach=np.array([np.cos(np.deg2rad(-140)),np.sin(np.deg2rad(-140)),0.])
            jaw=np.array([0.,0.,-1.])
            rot=np.column_stack((approach,jaw,np.cross(approach,jaw)))
            def side(xyz):
                t=np.eye(4);t[:3,:3]=rot;t[:3,3]=xyz
                return ArmCommand('right','cartesian_pose',transform_pose(t))
            self.book_contact=contact.copy();self.book_quat=tuple(side(contact).values[3:])
            add('open_both',grip('left',.07),grip('right',.045))
            add('left_above_upper_book',tcp('left',u+[0,0,.07]));add('left_stabilize_upper',tcp('left',u+[0,0,.031]))
            add('right_side_pregrasp',side(contact-approach*.10));add('right_middle_edge',side(contact))
            add('right_close_book',grip('right',.006));add('verify_grasp')
            add('pull_60mm',side(contact+axis*.06));add('pull_fully_130mm',side(contact+axis*.13));add('settle_neighbours')
            add('left_withdraw',tcp('left',u+[0,0,.07]))
            dest=contact+axis*.18;dest[2]=task.spec.parameters['table_z']+.012
            add('place_extracted_book',side(dest));add('release_book',grip('right',.045));add('verify_placement')
            add('right_withdraw',side(dest+axis*.08))
            add('home_both',ArmCommand('left','joint_position',(0.,)*6),ArmCommand('right','joint_position',(0.,)*6))
        return seq

class TimedBackend(IsaacSimBackend):
    def open(self, config):
        super().open(config)
        import signal
        signal.signal(signal.SIGINT,signal.default_int_handler)
    def step(self, duration):
        t=time.monotonic();super().step(duration)
        print('QUAL_TIMING step',round(time.monotonic()-t,3),flush=True)
        if a.task=='L2-03' and getattr(task,'scene',None) is not None:
            rows=task.scene.measure()
            upper=rows.get('upper_book',{})
            force=sum(upper.get('fingers',{}).get('left',[]))
            if force>8.:
                raise RuntimeError(f'Qualification upper-book contact guard: {force:.3f} N > engineering stop 8 N; not a calibrated real force limit')
    def capture_rgb(self):
        t=time.monotonic();frames=super().capture_rgb()
        print('QUAL_TIMING capture',round(time.monotonic()-t,3),flush=True)
        return frames
policy=Sequence()
run,result=run_episode(cfg,TimedBackend(),R5Adapter(),task,policy,a.output.parent,
    run_directory=a.output,record_video=True,preparation=preparation_action(cfg))
(a.output/'qualification-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=str))
print('QUALIFICATION_RESULT',json.dumps(result,default=str),flush=True)
