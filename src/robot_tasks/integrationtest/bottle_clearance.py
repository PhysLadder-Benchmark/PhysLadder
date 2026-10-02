"""Offline L3-03 workspace diagnostic; never imported by the robot policy.

Run in the planner environment (numpy/scipy/trimesh). Numerical IK only proposes
joint-limit-respecting inspection poses; it is NOT the production planner.
Conservative CAD/URDF boxes check sampled inter-arm and camera/prop clearance.
This is neither continuous path certification nor physical grasp/task success.
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from ..cameras.profile import load_camera_rig
from ..paths import PROJECT_ROOT
from ..robots.control import width_to_joints
from ..robots.kinematics import R5Kinematics
from ..robots.transforms import origin_transform, pose_residual, pose_transform, transform_pose
from ..robots.urdf import read_urdf
from ..registry import resolve_task
from ..tasks.bench.spec import build_spec


def box_vertices(transform, low, high):
    points = np.array([(x, y, z) for x in (low[0], high[0])
                       for y in (low[1], high[1]) for z in (low[2], high[2])])
    return points @ transform[:3, :3].T + transform[:3, 3]


def separation(first, second):
    """SAT separating-plane margin for two orthogonal boxes, in metres.

    Positive is a conservative lower bound on distance. Nonpositive means box
    overlap, which does not establish collision between the enclosed meshes.
    """
    a = np.array([first[i] - first[0] for i in (4, 2, 1)])
    b = np.array([second[i] - second[0] for i in (4, 2, 1)])
    axes = np.concatenate((a, b, np.cross(a[:, None], b).reshape(-1, 3)))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-10] / norms[norms > 1e-10, None]
    x, y = first @ axes.T, second @ axes.T
    return float(np.maximum(x.min(0) - y.max(0), y.min(0) - x.max(0)).max())


class Geometry:
    def __init__(self, config):
        import trimesh

        self.profile = config.robot
        self.kin = R5Kinematics(config.robot)
        self.urdf = read_urdf(self.profile.asset_path, dict(self.profile.package_roots))
        self.mesh_bounds = {}
        root = ET.parse(self.profile.asset_path).getroot()
        for link in root.findall('link'):
            collision = link.find('collision')
            node = collision.find('geometry/mesh')
            uri = node.get('filename')
            if uri.startswith('package://'):
                package, relative = uri[10:].split('/', 1)
                path = dict(self.profile.package_roots)[package] / relative
            else:
                path = self.profile.asset_path.parent / uri
            mesh = trimesh.load(path, force='mesh')
            mesh.apply_scale(tuple(map(float, node.get('scale', '1 1 1').split())))
            o = collision.find('origin')
            t = origin_transform(tuple(map(float, o.get('xyz', '0 0 0').split())),
                                 tuple(map(float, o.get('rpy', '0 0 0').split()))) if o is not None else np.eye(4)
            self.mesh_bounds[link.get('name')] = (t, mesh.bounds)
        self.rig = load_camera_rig(config.source.parent / config.raw['simulation']['camera_profile'])
        assets = PROJECT_ROOT / 'assets/cameras'
        self.housing_dims = np.array(json.loads((assets / 'gemini305.json').read_text())['dimensions_m'])
        self.bracket_dims = np.array(json.loads((assets / 'r5_wrist_camera_mount.json').read_text())['dimensions_m'])
        self.bracket_pose = pose_transform(json.loads((assets / 'gemini305_r5_mounted.json').read_text())['bracket_centred_cad_in_assembly_wxyz'])

    def boxes(self, arm, q, width):
        positions = dict(zip(self.profile.joint_names, q))
        positions.update(zip(self.profile.gripper.joint_names, width_to_joints(self.profile, width)))
        transforms = {self.profile.base_link: self.kin.bases[arm]}
        pending = list(self.urdf.joints)
        while pending:
            for joint in list(pending):
                if joint.parent not in transforms:
                    continue
                t = origin_transform(joint.origin_xyz, joint.origin_rpy)
                motion = np.eye(4)
                value = positions.get(joint.name, 0.0)
                if joint.kind == 'prismatic':
                    motion[:3, 3] = np.asarray(joint.axis) * value
                elif joint.kind in ('revolute', 'continuous'):
                    from ..robots.transforms import axis_rotation
                    motion[:3, :3] = axis_rotation(joint.axis, value)
                transforms[joint.child] = transforms[joint.parent] @ t @ motion
                pending.remove(joint)
        boxes = {name: box_vertices(transforms[name] @ t, *bounds)
                 for name, (t, bounds) in self.mesh_bounds.items()}
        camera = next(c for c in self.rig.cameras if c.parent == arm)
        optical = transforms[self.profile.ee_link] @ pose_transform(camera.optical_pose_wxyz)
        assembly = optical @ pose_transform(camera.asset_pose_wxyz)
        boxes['camera'] = box_vertices(assembly, -self.housing_dims / 2, self.housing_dims / 2)
        boxes['bracket'] = box_vertices(assembly @ self.bracket_pose, -self.bracket_dims / 2, self.bracket_dims / 2)
        return boxes, optical

    def solve(self, arm, pose, seed):
        from scipy.optimize import least_squares

        goal = pose_transform(pose)
        def residual(q):
            r = pose_residual(pose_transform(self.kin.forward(q, arm)), goal)
            r[3:] *= 0.2
            return r
        rng = np.random.default_rng(0)
        guesses = [seed, *rng.uniform(self.kin.lower, self.kin.upper, (24, 6))]
        for guess in guesses:
            r = least_squares(residual, guess, bounds=(self.kin.lower, self.kin.upper), max_nfev=150)
            if np.linalg.norm(residual(r.x)) < 1e-5:
                return r.x
        raise RuntimeError(f'No diagnostic IK pose: {arm} {pose}')


def check(output):
    _, config = resolve_task('L3-03')
    geometry = Geometry(config)
    spec = build_spec('L3-03')
    body, cap = spec.design.articles
    p = spec.parameters
    left_z = body.initial[2] + p['body_height'] * 0.45
    right_z = cap.initial[2] + 0.006
    home = {m.name: np.array(m.home_positions_rad) for m in config.robot.mounts}
    poses = {'left': transform_pose(origin_transform((body.initial[0], 0.020, left_z), (0, 0, -math.pi / 2))),
             'right': transform_pose(origin_transform((body.initial[0], 0, right_z), (0, 0, math.pi / 6)))}
    q = {arm: geometry.solve(arm, pose, home[arm]) for arm, pose in poses.items()}
    # The protocol repeats 60-degree turns with open-jaw vertical withdrawal and
    # yaw reset above the cap. Full thread progress still belongs to physics.
    samples = []
    for offset in np.linspace(0.08, 0, 17):
        samples.append(('body_approach', float(offset), 0.08, 0.0, 0.070, 0.070, 0.0))
    for width in np.linspace(0.070, p['body_width'], 11):
        samples.append(('body_close', 0.0, 0.08, 0.0, float(width), 0.070, 0.0))
    for cycle in range(12):
        rise = cycle * p['pitch'] / 6
        for offset in np.linspace(0.08, 0, 17):
            samples.append(('cap_approach', 0.0, float(offset), 0.0, p['body_width'], 0.070, rise))
        for width in np.linspace(0.070, 0.043, 10):
            samples.append(('cap_close', 0.0, 0.0, 0.0, p['body_width'], float(width), rise))
        for angle in np.linspace(0, math.pi / 3, 31):
            samples.append(('turn', 0.0, 0.0, float(angle), p['body_width'], 0.043,
                            rise + p['pitch'] * float(angle) / (2 * math.pi)))
        rise += p['pitch'] / 6
        for width in np.linspace(0.043, 0.070, 10):
            samples.append(('cap_open', 0.0, 0.0, math.pi / 3, p['body_width'], float(width), rise))
        for offset in np.linspace(0, 0.08, 17):
            samples.append(('regrasp_withdraw', 0.0, float(offset), math.pi / 3, p['body_width'], 0.070, rise))
        for angle in np.linspace(math.pi / 3, 0, 31):
            samples.append(('regrasp_yaw_reset', 0.0, 0.08, float(angle), p['body_width'], 0.070, rise))
    minima = {'inter_arm': (float('inf'), ''), 'camera_prop': (float('inf'), '')}
    failures, checkpoints = [], []
    previous_target = {}
    max_joint_step = 0.0
    for i, (stage, left_offset, right_offset, yaw, lw, rw, rise) in enumerate(samples):
        target = {'left': transform_pose(origin_transform((body.initial[0], 0.020 + left_offset, left_z), (0, 0, -math.pi / 2))),
                  'right': transform_pose(origin_transform((body.initial[0], 0, right_z + rise + right_offset), (0, 0, math.pi / 6 + yaw)))}
        for arm in q:
            if target[arm] != previous_target.get(arm):
                next_q = geometry.solve(arm, target[arm], q[arm])
                if i:
                    max_joint_step = max(max_joint_step, float(np.max(np.abs(next_q - q[arm]))))
                q[arm] = next_q
                previous_target[arm] = target[arm]
        left, lc = geometry.boxes('left', q['left'], lw)
        right, rc = geometry.boxes('right', q['right'], rw)
        for a, av in left.items():
            for b, bv in right.items():
                margin = separation(av, bv)
                if margin < minima['inter_arm'][0]:
                    minima['inter_arm'] = (margin, f'{stage}:{a}/{b}')
                if margin <= 0 and len(failures) < 20:
                    failures.append({'sample': i, 'stage': stage, 'pair': [a, b], 'margin_m': margin})
        prop_boxes = {'body': box_vertices(pose_transform(body.initial), (-p['body_width'] / 2, -p['body_width'] / 2, 0), (p['body_width'] / 2, p['body_width'] / 2, p['body_height'])),
                      'neck': box_vertices(pose_transform(body.initial), (-.015, -.015, p['body_height']), (.015, .015, p['neck_top'])),
                      'cap': box_vertices(origin_transform((cap.initial[0], 0, cap.initial[2] + rise), (0, 0, 0)), (-.0215, -.0215, -p['cap_half']), (.0215, .0215, p['cap_half']))}
        for arm, boxes in [('left', left), ('right', right)]:
            for name in ('camera', 'bracket'):
                for prop, vertices in prop_boxes.items():
                    margin = separation(boxes[name], vertices)
                    if margin < minima['camera_prop'][0]:
                        minima['camera_prop'] = (margin, f'{stage}:{arm}/{name}/{prop}')
                    if margin <= 0 and len(failures) < 20:
                        failures.append({'sample': i, 'stage': stage, 'pair': [arm, name, prop], 'margin_m': margin})
        if i in (0, 27, 54, 84) or i == len(samples) - 1:
            checkpoints.append({'sample': i, 'stage': stage, 'joints': {a: v.tolist() for a, v in q.items()},
                                'widths': {'left': lw, 'right': rw}, 'cap_rise_m': rise,
                                'camera_poses': {'left': transform_pose(lc), 'right': transform_pose(rc)}})
    result = {'passed': not failures and max_joint_step < 0.3, 'version': spec.design.version,
              'scope': 'Offline sampled IK and conservative CAD/URDF boxes; not dynamic grasp, continuous collision certification or task success. No benchmark controller or prompt receives these poses.',
              'limitations': 'Provisional camera registration; no self-collision test or production planner acceptance. Box overlap can be a false positive.',
              'samples': len(samples), 'minimum_separation_m': minima,
              'max_sample_joint_step_rad': max_joint_step, 'failures': failures, 'checkpoints': checkpoints}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('checkpoints',)}, indent=2))
    return result


def render_checkpoints(audit_path):
    """Isaac-only static diagnostic poses, never a task execution or score."""
    from PIL import Image
    from ..backends.isaacsim import IsaacSimBackend
    from ..config import ExecutionSettings
    from ..robots.adapters import R5Adapter

    report = json.loads(audit_path.read_text(encoding='utf-8'))
    output = audit_path.with_suffix('')
    output.mkdir(parents=True, exist_ok=True)
    spec, config = resolve_task('L3-03')
    if report['version'] != build_spec('L3-03').design.version:
        raise ValueError('Workspace report belongs to a different prop version')
    config = spec.factory.prepare_config(config)
    backend, robot, task = IsaacSimBackend(), R5Adapter(), spec.factory()
    backend.run_directory = output
    try:
        backend.open(config)
        robot.config, robot.backend = config, backend
        robot.execution = ExecutionSettings.from_config(config)
        robot.kinematics = R5Kinematics(config.robot)
        robot.instances = backend.add_r5(config.robot)
        task.build_scene(backend, config)
        backend.reset(config.seed)
        task.reset(backend, config.seed)
        robot.reset(config.seed)
        from isaacsim.core.utils.types import ArticulationAction
        for row in report['checkpoints']:
            for arm, joints in row['joints'].items():
                target = np.r_[joints, width_to_joints(config.robot, row['widths'][arm])]
                instance = robot.instances[arm]
                instance.set_joint_positions(target, joint_indices=robot.indices[arm])
                instance.set_joint_velocities(np.zeros(8), joint_indices=robot.indices[arm])
                instance.apply_action(ArticulationAction(joint_positions=target, joint_indices=robot.indices[arm]))
            # Explicitly authored inspection pose; no manipulation/success claim.
            for name, pose in task.scene.initial.items():
                xyz = np.asarray(pose[:3]).copy()
                if name == 'cap':
                    xyz[2] += row['cap_rise_m']
                task.scene.bodies[name].set_world_pose(xyz, np.asarray(pose[3:]))
                task.scene.bodies[name].set_linear_velocity(np.zeros(3))
                task.scene.bodies[name].set_angular_velocity(np.zeros(3))
            backend.step(config.physics_dt_s)
            backend.capture_rgb()  # Flush the changed inspection pose.
            for frame in backend.capture_rgb():
                Image.frombytes('RGB', (frame.width, frame.height), frame.data).save(
                    output / f"{row['sample']:04d}_{row['stage']}_{frame.camera}.png")
        (output / 'scope.txt').write_text(
            'Authored diagnostic poses for visual inspection only. Not physical grasp, planned motion or GPT success evidence.\n',
            encoding='utf-8')
    finally:
        robot.close()
        backend.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=PROJECT_ROOT / 'runs/l3_03_square_clearance.json')
    parser.add_argument('--render', action='store_true', help='Render an existing audit in Isaac; no model calls')
    args = parser.parse_args()
    if args.render:
        render_checkpoints(args.output)
    else:
        raise SystemExit(0 if check(args.output)['passed'] else 2)
