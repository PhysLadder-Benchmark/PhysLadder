"""Eight passive physical scenes. Pose writes occur only in episode reset."""

from dataclasses import asdict
import numpy as np

from ...robots.transforms import pose_transform, origin_transform, transform_pose
from .assets import import_asset
from .specs import build_design, local_corners


class L1Scene:
    root = "/World/L1"

    def __init__(self, backend, config, settings):
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.prims import SingleRigidPrim, RigidPrim
        from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade

        self.backend, self.config, self.settings = backend, config, settings
        self.design = build_design(config.task_id)
        self.bodies, self.contacts = {}, {}
        self.material = PhysicsMaterial(
            self.root + "/Material", static_friction=0.8, dynamic_friction=0.8, restitution=0.0
        )
        stage = backend.world.stage
        self.stage = stage
        self.colliders = []
        evidence = []
        # Build at origin: normalization must not include the world placement.
        UsdGeom.Xform.Define(stage, self.root + "/Fixture")
        if self.design.fixture_asset:
            paths, record = import_asset(
                stage,
                self.root + "/Fixture",
                self.design.fixture_asset,
                self.design.fixture_size,
                (0, 0, 0),
                dynamic=False,
            )
            self.colliders.extend(paths)
            evidence.append(record)
            self.set_pose(self.root + "/Fixture", self.design.fixture_pose)
        for part in self.design.fixtures:
            self.add_part(self.root + "/Fixture/" + part.name, part)
        for article in self.design.articles:
            path = self.root + "/" + article.name
            UsdGeom.Xform.Define(stage, path)
            if article.asset:
                paths, record = import_asset(
                    stage, path, article.asset, article.asset_size, article.asset_rotation, dynamic=True
                )
                self.colliders.extend(paths)
                evidence.append(record)
            else:
                for part in article.parts:
                    self.add_part(path + "/" + part.name, part)
            prim = stage.GetPrimAtPath(path)
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(article.mass)
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateSolverPositionIterationCountAttr(32)
            body.CreateSolverVelocityIterationCountAttr(0)
            self.bodies[article.name] = backend.world.scene.add(
                SingleRigidPrim(
                    path,
                    name="l1_" + article.name,
                    position=np.array(article.initial[:3]),
                    orientation=np.array(article.initial[3:]),
                )
            )
        # Match master interaction friction/contact offsets on the bench and R5.
        paths = (
            self.colliders
            + ["/World/Bench"]
            + [p for a in backend.scene_report["arms"].values() for p in a["colliders"]]
        )
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            while prim.IsInstanceProxy():
                ancestor = prim.GetParent()
                while ancestor and not ancestor.IsInstance():
                    ancestor = ancestor.GetParent()
                if not ancestor:
                    raise RuntimeError(f"Cannot deinstance collider {path}")
                ancestor.SetInstanceable(False)
                prim = stage.GetPrimAtPath(path)
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(0.001)
            api.CreateRestOffsetAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                self.material.material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
        arm = config.arms[0]
        base = next(m.prim_path for m in config.robot.mounts if m.name == arm)
        self.fingers = RigidPrim(
            base + "/link[78]",
            name="l1_finger_contacts",
            reset_xform_properties=False,
            track_contact_forces=True,
            contact_filter_prim_paths_expr=[self.root + "/" + a.name for a in self.design.articles],
            max_contact_count=512,
            disable_stablization=False,
        )
        self.support_filters = [p for p in self.colliders if p.startswith(self.root + "/Fixture/")] + [
            "/World/Bench"
        ]
        self.supports = {
            a.name: RigidPrim(
                self.root + "/" + a.name,
                name="l1_support_" + a.name,
                reset_xform_properties=False,
                track_contact_forces=True,
                contact_filter_prim_paths_expr=self.support_filters,
                max_contact_count=512,
                disable_stablization=False,
            )
            for a in self.design.articles
        }
        backend.scene_report["l1"] = {
            "version": "l1-rigid-v1",
            "design": asdict(self.design),
            "assets": evidence,
            "scoring": asdict(settings),
            "colliders": self.colliders,
            "ground_truth_use": "evaluator only",
            "controller": {
                "adapter": "R5Adapter / numerical IK",
                "motion_mode": config.raw.get("execution", {}).get("motion_mode", "direct_ik"),
                "gripper_timing": config.raw.get("execution", {}).get("gripper_timing", "simultaneous"),
            },
            "physics_variant": "CPU PhysX, fixed mesh cavities, dynamic convex decomposition",
            "object_attachment": False,
            "initial_poses": {},
        }

    def set_pose(self, path, pose):
        from pxr import UsdGeom, Gf

        x = UsdGeom.Xformable(self.stage.GetPrimAtPath(path))
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
        x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(pose[3], Gf.Vec3d(*pose[4:])))

    def add_part(self, path, part):
        from pxr import UsdGeom, UsdPhysics, Gf

        shape = (
            UsdGeom.Cube.Define(self.stage, path)
            if part.shape == "box"
            else UsdGeom.Cylinder.Define(self.stage, path)
        )
        if part.shape == "box":
            shape.CreateSizeAttr(1.0)
        else:
            shape.CreateRadiusAttr(part.size[0])
            shape.CreateHeightAttr(part.size[2])
            shape.CreateAxisAttr("Z")
        self.set_pose(path, transform_pose(origin_transform(part.position, part.rotation)))
        if part.shape == "box":
            UsdGeom.Xformable(shape).AddScaleOp().Set(Gf.Vec3f(*part.size))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*part.color)])
        if part.collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            self.colliders.append(path)

    def reset(self, seed):
        rng = np.random.default_rng(seed)
        self.initial = {}
        for a in self.design.articles:
            t = pose_transform(a.initial)
            t[:2, 3] += rng.uniform(-self.settings.position_jitter_m, self.settings.position_jitter_m, 2)
            yaw = rng.uniform(-self.settings.yaw_jitter_rad, self.settings.yaw_jitter_rad)
            t[:3, :3] = origin_transform((0, 0, 0), (0, 0, yaw))[:3, :3] @ t[:3, :3]
            p = transform_pose(t)
            self.initial[a.name] = p
            self.bodies[a.name].set_default_state(
                position=np.array(p[:3]),
                orientation=np.array(p[3:]),
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
            )
        self.backend.reset(seed)
        self.fingers.initialize()
        for view in self.supports.values():
            view.initialize()
        self.backend.scene_report["l1"]["initial_poses"] = self.initial

    def measure(self):
        dt = self.config.physics_dt_s
        forces = np.asarray(self.fingers.get_contact_force_matrix(dt=dt))
        if forces.shape != (2, len(self.bodies), 3) or not np.isfinite(forces).all():
            raise RuntimeError("Invalid L1 finger contact data")
        contact_data = self.fingers.get_contact_force_data(dt=dt)
        points = np.asarray(contact_data[1])
        counts = np.asarray(contact_data[4])
        starts = np.asarray(contact_data[5])
        result = {}
        for i, a in enumerate(self.design.articles):
            support = np.asarray(self.supports[a.name].get_contact_force_matrix(dt=dt))
            if support.shape != (1, len(self.support_filters), 3) or not np.isfinite(support).all():
                raise RuntimeError("Invalid L1 support contact data")
            body = self.bodies[a.name]
            xyz, q = body.get_world_pose()
            t = pose_transform((*xyz, *q))
            local_contacts = []
            for f in range(2):
                n = int(counts[f, i])
                start = int(starts[f, i])
                local_contacts.append(((points[start : start + n] - xyz) @ t[:3, :3]).tolist())
            result[a.name] = {
                "pose": [*map(float, xyz), *map(float, q)],
                "linear_velocity": body.get_linear_velocity().tolist(),
                "angular_velocity": body.get_angular_velocity().tolist(),
                "finger_forces": np.linalg.norm(forces[:, i, :], axis=1).tolist(),
                "finger_contacts_local": local_contacts,
                "fixture_force_n": float(np.linalg.norm(support[0, :-1].sum(axis=0))),
                "bench_force_n": float(np.linalg.norm(support[0, -1])),
                "corners": (local_corners(a) @ t[:3, :3].T + xyz).tolist(),
            }
        return result
