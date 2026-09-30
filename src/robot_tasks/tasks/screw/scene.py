"""Passive coarse screw scene. Object poses are written only at episode reset."""

from dataclasses import asdict

import numpy as np

from ...robots.transforms import pose_transform
from .evaluation import SCORE_VERSION
from .geometry import helix_pieces, screw_surface, socket_shell_mesh
from .spec import SPEC


class ScrewScene:
    root = "/World/ScrewTask"

    def __init__(self, backend, config):
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.prims import RigidPrim, SingleRigidPrim
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        self.backend, self.config = backend, config
        self.stage = backend.world.stage
        self.material = PhysicsMaterial(
            self.root + "/GripMaterial",
            static_friction=0.8,
            dynamic_friction=0.8,
            restitution=0.0,
        )
        self.thread_material = PhysicsMaterial(
            self.root + "/ThreadMaterial",
            static_friction=SPEC.friction,
            dynamic_friction=SPEC.friction,
            restitution=0.0,
        )
        PhysxSchema.PhysxMaterialAPI.Apply(
            self.thread_material.material.GetPrim()
        ).CreateFrictionCombineModeAttr("min")
        path = self.root + "/Screw"
        prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.09)
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rigid.CreateSolverPositionIterationCountAttr(48)
        rigid.CreateSolverVelocityIterationCountAttr(4)
        self.cylinder(
            path + "/Core",
            SPEC.core_radius,
            SPEC.shaft_length,
            (0, 0, SPEC.shaft_length / 2),
            (0.8, 0.46, 0.12),
        )
        self.cylinder(
            path + "/Head",
            SPEC.head_radius,
            SPEC.head_height,
            (0, 0, SPEC.shaft_length + SPEC.head_height / 2),
            (0.9, 0.28, 0.05),
        )
        for i, (vertices, faces) in enumerate(helix_pieces()):
            self.mesh(
                path + f"/Thread_{i}",
                vertices,
                faces,
                convex=True,
                color=(0.92, 0.62, 0.16),
            )
        # A visible radial mark helps judge actual screw rotation, not only wrist rotation.
        self.box(
            path + "/RotationMark",
            (0.018, 0.004, 0.0004),
            (0.011, 0, SPEC.height + 0.0002),
            (0.1, 0.1, 0.1),
            collision=False,
        )
        self.body = backend.world.scene.add(SingleRigidPrim(path, name="coarse_screw"))
        fixture = self.root + "/Socket"
        UsdGeom.Xform.Define(self.stage, fixture).AddTranslateOp().Set(
            (*SPEC.socket_xy, 0.0)
        )
        vertices, faces = socket_shell_mesh()
        self.mesh(
            fixture + "/OuterShell",
            vertices,
            faces,
            convex=False,
            color=(0.15, 0.35, 0.8),
        )
        self.box(
            fixture + "/Floor",
            (0.080, 0.080, SPEC.socket_floor),
            (0, 0, SPEC.socket_floor / 2),
            (0.1, 0.25, 0.65),
            thread=True,
        )
        self.fixture_paths = (fixture + "/OuterShell", fixture + "/Floor")
        thread_paths = []
        for i, (vertices, faces) in enumerate(helix_pieces(female=True)):
            part_path = fixture + f"/ContactThread_{i}"
            # Static convex pieces avoid edge artifacts of concave triangle
            # mesh contact with the compound male thread on CPU PhysX.
            self.mesh(part_path, vertices, faces, convex=True, color=(0.15, 0.35, 0.8))
            thread_paths.append(part_path)
        self.contact_paths = (*self.fixture_paths, *thread_paths, "/World/Bench")
        self.fingers = RigidPrim(
            next(m.prim_path for m in config.robot.mounts if m.name == "left")
            + "/link[78]",
            name="screw_finger_contacts",
            reset_xform_properties=False,
            track_contact_forces=True,
            contact_filter_prim_paths_expr=[path],
            max_contact_count=2048,
            disable_stablization=False,
        )
        self.supports = RigidPrim(
            path,
            name="screw_support_contacts",
            reset_xform_properties=False,
            track_contact_forces=True,
            contact_filter_prim_paths_expr=list(self.contact_paths),
            max_contact_count=4096,
            disable_stablization=False,
        )
        backend.planning_fixture_paths = self.fixture_paths
        backend.planning_payloads = self.planning_payloads
        backend.scene_report["l3_05"] = {
            "spec": asdict(SPEC),
            "initial_pose": SPEC.initial_pose,
            "source": "project-authored coarse teaching screw; no external asset download",
            "reference": "https://github.com/isaac-sim/IsaacGymEnvs/blob/main/docs/factory.md",
            "physics": "CPU PhysX; convex male and static female helix segments, open annular shell",
            "object_attachment": False,
            "screw_joint": False,
            "initially_engaged": False,
            "calibration": "engineering geometry/contact parameters, not a standard commercial fastener",
            "damage_model": "none; stripping, plastic deformation and breakage are not simulated",
            "model_observation": "RGB-D, camera parameters and robot state; no object truth or force/torque sensor",
            "score_version": SCORE_VERSION,
            "score": "pickup, net held rotation and descent bounded by thread lead, insertion depth and released stable seating",
            "planning_contact_boundary": "outer shell and floor checked; mating thread teeth excluded from planning obstacles to permit intended contact; all teeth collide in physics",
        }

    def collision(self, prim, *, thread=False, convex=False):
        from pxr import PhysxSchema, UsdPhysics, UsdShade

        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(0.0003)
        api.CreateRestOffsetAttr(0.0)
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(
                "convexHull" if convex else "none"
            )
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            (self.thread_material if thread else self.material).material,
            materialPurpose="physics",
        )

    def mesh(self, path, vertices, faces, *, convex, color):
        from pxr import Gf, UsdGeom

        mesh = UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        self.collision(mesh.GetPrim(), thread=True, convex=convex)

    def cylinder(self, path, radius, height, position, color):
        from pxr import Gf, UsdGeom

        shape = UsdGeom.Cylinder.Define(self.stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.CreateAxisAttr("Z")
        shape.AddTranslateOp().Set(Gf.Vec3d(*position))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        self.collision(shape.GetPrim())

    def box(self, path, size, position, color, *, collision=True, thread=False):
        from pxr import Gf, UsdGeom

        shape = UsdGeom.Cube.Define(self.stage, path)
        shape.CreateSizeAttr(1.0)
        shape.AddTranslateOp().Set(Gf.Vec3d(*position))
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            self.collision(shape.GetPrim(), thread=thread)

    def reset(self, seed):
        self.body.set_default_state(
            position=np.array(SPEC.initial_pose[:3]),
            orientation=np.array(SPEC.initial_pose[3:]),
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
        )
        self.backend.reset(seed)
        self.fingers.initialize()
        self.supports.initialize()

    def measure(self):
        dt = self.config.physics_dt_s
        fingers = np.asarray(self.fingers.get_contact_force_matrix(dt=dt))
        supports = np.asarray(self.supports.get_contact_force_matrix(dt=dt))
        if (
            fingers.shape != (2, 1, 3)
            or supports.shape != (1, len(self.contact_paths), 3)
            or not np.isfinite(fingers).all()
            or not np.isfinite(supports).all()
        ):
            raise RuntimeError("Invalid screw contact measurements")
        p, q = self.body.get_world_pose()
        return {
            "pose": [*map(float, p), *map(float, q)],
            "linear_velocity": self.body.get_linear_velocity().tolist(),
            "angular_velocity": self.body.get_angular_velocity().tolist(),
            "finger_forces": np.linalg.norm(fingers[:, 0], axis=1).tolist(),
            "fixture_force_n": float(np.linalg.norm(supports[0, :-1].sum(axis=0))),
            "bench_force_n": float(np.linalg.norm(supports[0, -1])),
        }

    def planning_payloads(self):
        row = self.measure()
        if not all(f > SPEC.contact_threshold for f in row["finger_forces"]):
            return {"left": []}
        t = pose_transform(row["pose"])
        return {
            "left": [
                {
                    "surface_points_world": (
                        screw_surface() @ t[:3, :3].T + t[:3, 3]
                    ).tolist(),
                    "source": "measured bilateral contact; actual thread surface, collision only",
                }
            ]
        }
