"""Check local upstream Franka articulation, joint actions, and RGB rendering.

This is an integration smoke check, not a RoboDojo task or an R5 adaptation.
"""
import argparse
import csv
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
project = Path(__file__).resolve().parents[1]
asset = project / 'assets/robodojo/Robots/franka/Franka.usd'
if not asset.exists():
    raise FileNotFoundError(asset)

from isaacsim import SimulationApp
from runtime_config import simulation_config
app = SimulationApp(simulation_config())

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdLux
from PIL import Image

world = World(stage_units_in_meters=1.0, physics_dt=1/240, rendering_dt=1/60)
world.scene.add_default_ground_plane()
add_reference_to_stage(str(asset), '/World/Robot')
robot = world.scene.add(SingleArticulation('/World/Robot', name='franka'))
q0 = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04], dtype=np.float32)
UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(900)
camera = rep.create.camera(position=(1.6, -1.6, 1.3), look_at=(0.3, 0, 0.4), clipping_range=(0.005, 100))
product = rep.create.render_product(camera, (960, 720))
rgb = rep.AnnotatorRegistry.get_annotator('rgb')
rgb.attach(product)
rep.orchestrator.set_capture_on_play(False)
world.reset()
# One initial state assignment before the rollout; subsequent movement uses
# drive targets and physics exclusively.
robot.set_joint_positions(q0)
robot.set_joint_velocities(np.zeros(9, dtype=np.float32))
robot.set_joints_default_state(positions=q0)
rows, frames = [], []
for step in range(960):
    t = step/240
    target = q0.copy()
    target[0] = 0.3 * np.sin(np.pi*t/2)
    target[7:] = 0.04 if t < 2 else 0.01
    robot.apply_action(ArticulationAction(joint_positions=target))
    world.step(render=False)
    q = np.asarray(robot.get_joint_positions())
    rows.append([t, *q.tolist(), *target.tolist()])
    if step % 12 == 0:
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=2)
        pixels = rgb.get_data()
        if pixels is not None and np.asarray(pixels).size:
            frames.append(Image.fromarray(np.asarray(pixels)[..., :3].astype(np.uint8)))
with (a.output/'joints.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time_s', *[f'q{i}' for i in range(9)], *[f'target{i}' for i in range(9)]])
    writer.writerows(rows)
values = np.asarray(rows)
joint_span = float(np.ptp(values[:, 1]))
finger_span = float(np.ptp(values[:, 8]))
frame_change = float(np.mean(np.abs(np.asarray(frames[5], dtype=float)-np.asarray(frames[-1], dtype=float)))) if len(frames)>5 else 0
result = {'asset': str(asset), 'dof_names': robot.dof_names, 'frames': len(frames),
          'joint_1_range_rad': joint_span, 'finger_range_m': finger_span,
          'frame_pixel_change': frame_change,
          'passed': bool(np.isfinite(values).all() and joint_span>0.2 and finger_span>0.02 and frame_change>0.1),
          'scope': 'Franka joint control and RGB smoke only; no IK, no RoboDojo episode, no R5, no GPT'}
if frames:
    frames[0].save(a.output/'first_frame.png')
    frames[-1].save(a.output/'last_frame.png')
    frames[0].save(a.output/'rollout.gif', save_all=True, append_images=frames[1:], duration=50, loop=0)
(a.output/'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print('ROBOT_SMOKE_RESULT '+json.dumps(result), flush=True)
app.close()
raise SystemExit(0 if result['passed'] else 2)
