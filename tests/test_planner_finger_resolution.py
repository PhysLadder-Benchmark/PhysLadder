import os,unittest
from unittest.mock import patch
from robot_tasks.planning.assets import requested_finger_spheres
class PlannerFingerResolutionTests(unittest.TestCase):
    def test_legacy_default(self):
        with patch.dict(os.environ,{},clear=True):self.assertEqual(requested_finger_spheres(),12)
    def test_opt_in_refinement(self):
        with patch.dict(os.environ,{'ROBOT_TASKS_PLANNER_FINGER_SPHERES':'64'}):self.assertEqual(requested_finger_spheres(),64)
    def test_invalid_range(self):
        for value in ('0','7','65','-1'):
            with self.subTest(value=value),patch.dict(os.environ,{'ROBOT_TASKS_PLANNER_FINGER_SPHERES':value}):
                with self.assertRaises(ValueError):requested_finger_spheres()
    def test_non_integer(self):
        for value in ('nan','12.5',''):
            with self.subTest(value=value),patch.dict(os.environ,{'ROBOT_TASKS_PLANNER_FINGER_SPHERES':value}):
                with self.assertRaises(ValueError):requested_finger_spheres()
