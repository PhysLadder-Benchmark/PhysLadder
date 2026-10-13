"""Native delivery integration: compound geometry, spring contacts and source/target identity."""
import unittest
from pathlib import Path
import numpy as np
from robot_tasks.asset_delivery import verified_prop
from robot_tasks.tasks.l1.assets import import_tuojing_asset
from robot_tasks.tasks.bench.native_geometry import verify_stage
try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None

@unittest.skipIf(Usd is None, 'USD libraries required')
class FeishuDeliveryTests(unittest.TestCase):
    def scene(self, prop, task, dynamic):
        try:
            path,_=verified_prop(prop,task_id=task)
        except (FileNotFoundError, ValueError) as exc:
            if 'Missing or modified asset' in str(exc):
                self.skipTest('Private Feishu delivery is not installed')
            raise
        stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/Prop')
        colliders,record=import_tuojing_asset(stage,'/Prop',prop,dynamic=dynamic,task_id=task)
        return stage,colliders,record

    def test_closed_marker_preserves_body_and_cap_geometry_without_nested_bodies(self):
        stage,colliders,record=self.scene('capped_marker_20261011','L1-01',True)
        verify_stage(stage,[('capped_marker_20261011','/Prop')])
        self.assertTrue(any('/Body/Colliders/' in p for p in colliders))
        self.assertTrue(any('/Cap/Colliders/' in p for p in colliders))
        self.assertFalse(any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()))
        self.assertFalse(any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse()))
        self.assertIn('closed-cap',record['overrides'][0])

    def test_strip_keeps_twelve_spring_contacts_with_fixed_housing(self):
        stage,colliders,_=self.scene('power_strip_20261010','L1-03',False)
        housing=stage.GetPrimAtPath('/Prop/Housing')
        self.assertTrue(UsdPhysics.RigidBodyAPI(housing).GetKinematicEnabledAttr().Get())
        self.assertFalse(housing.GetAttribute('physxRigidBody:enableCCD').Get())
        bodies=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        joints=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
        self.assertEqual(len(bodies),13);self.assertEqual(len(joints),12)
        for joint in joints:
            for rel in (UsdPhysics.Joint(joint).GetBody0Rel(),UsdPhysics.Joint(joint).GetBody1Rel()):
                self.assertTrue(all(stage.GetPrimAtPath(p) for p in rel.GetTargets()))
        for body in bodies:
            if body==housing:continue
            self.assertFalse(UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get())
            contact=stage.GetPrimAtPath(str(body.GetPath())+'/Contact')
            self.assertAlmostEqual(contact.GetAttribute('physxCollision:contactOffset').Get(),.00004)

    def test_millimetre_charger_points_remain_metre_sized_and_match_scoring(self):
        stage,colliders,_=self.scene('usb_charger_20261010','L1-03',True)
        verify_stage(stage,[('usb_charger_20261010','/Prop')])
        cache=UsdGeom.XformCache()
        from pxr import Gf
        for name in ('Blade0','Blade1'):
            mesh=UsdGeom.Mesh(stage.GetPrimAtPath('/Prop/Colliders/'+name))
            matrix=cache.GetLocalToWorldTransform(mesh.GetPrim())
            points=np.asarray([matrix.Transform(Gf.Vec3d(*p)) for p in mesh.GetPointsAttr().Get()])
            np.testing.assert_allclose(np.ptp(points,axis=0),[.0015,.0063,.018],atol=1e-8)

    def test_shallow_holder_visual_and_collision_floors_match_without_sealing_blade_hole(self):
        stage,colliders,record=self.scene('plug_storage_base','L1-03',True)
        verify_stage(stage,[('plug_storage_base','/Prop')])
        visual=UsdGeom.Mesh(stage.GetPrimAtPath('/Prop/Visuals/PocketFloor'))
        visual_points=np.asarray(visual.GetPointsAttr().Get())
        collision_points=np.concatenate([
            np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get())
            for path in colliders if '/PocketFloor_' in path])
        for points in (visual_points,collision_points):
            np.testing.assert_allclose([points[:,2].min(),points[:,2].max()],[.0442,.045],atol=1e-8)
            self.assertGreater(np.linalg.norm(points[:,:2],axis=1).min(),.0109)
        top=UsdGeom.Mesh(stage.GetPrimAtPath('/Prop/Visuals/TopFlange'))
        self.assertAlmostEqual(np.asarray(top.GetPointsAttr().Get())[:,2].max(),.055)
        self.assertTrue(any('20 -> 45 mm' in item for item in record['overrides']))

    def test_all_new_l1_entries_have_resolvable_dependency_paths(self):
        from pxr import UsdUtils
        from robot_tasks.asset_layout import bundles
        from robot_tasks.asset_delivery import ROOT
        for bundle in bundles(ROOT):
            if not bundle.package.startswith('l1_'):continue
            path=bundle.destination/bundle.entry
            if not path.exists():self.skipTest('Private delivery not installed')
            _,_,missing=UsdUtils.ComputeAllDependencies(str(path))
            self.assertEqual(missing,[],str(path))

    def test_visor_photo_variant_scales_visuals_colliders_and_inertia_together(self):
        from robot_tasks.tasks.l1.assets import VISOR_PHOTO_SCALE
        stage,colliders,record=self.scene('yellow_visor_20261011','L1-07',True)
        verify_stage(stage,[('yellow_visor_20261011','/Prop')])
        path,_=verified_prop('yellow_visor_20261011',task_id='L1-07')
        source=Usd.Stage.Open(str(path));root=source.GetDefaultPrim()
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):continue
            relative=str(prim.GetPath().MakeRelativePath(stage.GetPrimAtPath('/Prop').GetPath()))
            original=source.GetPrimAtPath(str(root.GetPath())+'/'+relative)
            np.testing.assert_allclose(UsdGeom.Mesh(prim).GetPointsAttr().Get(),
                                       np.asarray(UsdGeom.Mesh(original).GetPointsAttr().Get())*VISOR_PHOTO_SCALE,
                                       atol=1e-8)
        mass=UsdPhysics.MassAPI(stage.GetPrimAtPath('/Prop'));old=UsdPhysics.MassAPI(root)
        np.testing.assert_allclose(mass.GetCenterOfMassAttr().Get(),
                                   np.asarray(old.GetCenterOfMassAttr().Get())*VISOR_PHOTO_SCALE,atol=1e-8)
        np.testing.assert_allclose(mass.GetDiagonalInertiaAttr().Get(),
                                   np.asarray(old.GetDiagonalInertiaAttr().Get())*VISOR_PHOTO_SCALE**2,rtol=1e-6)
        self.assertEqual(len(colliders),208)
        self.assertTrue(any('80%' in item for item in record['overrides']))
        self.assertFalse(record['source_modified'])
