"""A released orange at the obsolete 10 cm rim must not count as returned."""
import unittest
from pathlib import Path
from copy import deepcopy
from robot_tasks.registry import resolve_task
from robot_tasks.tasks.bench.arm_selection import resolve_bench_spec
from robot_tasks.tasks.bench.orange_evaluation import OrangeEvaluator
from test_native_l2_scoring import soft_row
class MeasuredBoxReturn(unittest.TestCase):
    def test_floating_at_old_rim_is_not_returned(self):
        _,c=resolve_task('L2-06',depth=False,config_path=Path('configs/tasks/l2_06_real2sim.local.toml'))
        s=resolve_bench_spec(c.raw['task'],c.robot);rows=soft_row(s);e=OrangeEvaluator(s);e.update(0.,rows);e.lifted=True
        floating=deepcopy(rows);dz=s.parameters['table_z']+.1-s.parameters['support_z']
        for key in ['bounds_min','bounds_max','centroid','collision_bounds_min','collision_bounds_max']:floating['orange'][key][2]+=dz
        self.assertEqual(e.update(.1,floating).status,'running')
        self.assertEqual(e.update(.2,rows).status,'succeeded')
if __name__=='__main__':unittest.main()
