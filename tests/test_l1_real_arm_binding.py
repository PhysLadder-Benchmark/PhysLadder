"""Physical arm identity must agree across geometry, cameras and model metadata."""
from dataclasses import replace
import unittest

from robot_tasks.registry import resolve_task
from robot_tasks.cameras.profile import load_camera_rig, bind_wrist_cameras
from robot_tasks.cameras.geometry import project_world
from robot_tasks.cameras.overview import override_overview
from robot_tasks.backends.settings import SimulationSettings
from robot_tasks.robots.kinematics import R5Kinematics
from robot_tasks.policies.operating_notes import operating_notes
from robot_tasks.tasks.bench.arm_selection import resolve_bench_spec


class RealArmBindingTests(unittest.TestCase):
    def test_l1_arms_project_to_real_video_sides_and_keep_physical_calibration(self):
        for task in ('L1-01', 'L1-03', 'L1-05', 'L1-07', 'L1-08', 'L1-09'):
            with self.subTest(task=task):
                _, c = resolve_task(task)
                path = c.source.parent / c.raw['simulation']['camera_profile']
                original = load_camera_rig(path)
                bound = load_camera_rig(path, robot_profile=c.robot)
                self.assertEqual(bound.cameras[0], original.cameras[0])
                self.assertEqual(bind_wrist_cameras(bound, c.robot), bound)
                effective = override_overview(bound, SimulationSettings.from_config(c))
                overview = effective.select(('overview',))[0]
                mounts = {m.name: m for m in c.robot.mounts}
                pixels = {name: project_world([m.base_pose_wxyz[:3]], overview.optical_pose_wxyz,
                                             overview.intrinsics)[0][0] for name, m in mounts.items()}
                self.assertGreater(pixels['left'], pixels['right'])
                for name, physical in (('left', 'Right'), ('right', 'Left')):
                    self.assertEqual(mounts[name].prim_path, '/World/Robots/R5_' + physical)
                    camera = bound.select((name + '_wrist',))[0]
                    donor = next(x for x in original.cameras if x.existing_prim_path == camera.existing_prim_path)
                    self.assertEqual(camera, replace(donor, name=name + '_wrist', parent=name))
                    self.assertTrue(camera.existing_prim_path.startswith(mounts[name].prim_path + '/link6/'))
                notes = operating_notes(c)
                self.assertEqual(notes['arm_identity']['prim_by_arm']['left'], mounts['left'].prim_path)

    def test_common_l2_and_l3_profiles_and_cameras_keep_original_bindings(self):
        for task in ('L2-01', 'L2-03', 'L3-03', 'L3-14'):
            _, c = resolve_task(task)
            path = c.source.parent / c.raw['simulation']['camera_profile']
            self.assertEqual(load_camera_rig(path, robot_profile=c.robot), load_camera_rig(path))
            self.assertTrue(c.robot.mounts[0].prim_path.endswith('R5_Left'))
            self.assertNotIn('arm_identity', operating_notes(c))

    def test_first_cup_and_manipulated_objects_are_on_real_left_arm_side(self):
        for task, name in (('L1-01', 'marker'), ('L1-03', 'plug'), ('L1-07', 'hat'),
                           ('L1-08', 'lower_cup'), ('L1-09', 'rose')):
            _, c = resolve_task(task)
            spec = resolve_bench_spec(c.raw['task'], c.robot)
            obj = next(a for a in spec.design.articles if a.name == name)
            bases = {m.name: m.base_pose_wxyz[0] for m in c.robot.mounts}
            self.assertLess(abs(obj.initial[0]-bases['left']), abs(obj.initial[0]-bases['right']))
            if task == 'L1-08':
                second = next(a for a in spec.design.articles if a.name == 'upper_cup')
                self.assertLess(second.initial[0], obj.initial[0])
                self.assertGreater(spec.parameters['handoff_start_xy'][0], spec.parameters['handoff_end_xy'][0])

    def test_kinematics_follow_the_rebound_physical_arm(self):
        from robot_tasks.robots.profile import load_robot_profile
        _, c = resolve_task('L1-01')
        original = load_robot_profile(c.robot.source.with_name('r5_tuojing_lab.toml'))
        real, old = R5Kinematics(c.robot), R5Kinematics(original)
        q = (.1, -.2, .3, -.1, .2, -.3)
        self.assertEqual(real.forward(q, 'left'), old.forward(q, 'right'))
        self.assertEqual(real.forward(q, 'right'), old.forward(q, 'left'))


if __name__ == '__main__':
    unittest.main()
