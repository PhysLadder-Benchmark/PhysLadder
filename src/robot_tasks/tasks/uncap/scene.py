"""Local procedural marker: hollow cap, compliant friction sleeve, no latch joint.

Object poses are assigned only during initialization. During model operation all
motion/separation comes from PhysX contacts; no distance-triggered detachment.
"""

import math
from dataclasses import asdict

import numpy as np

from ...robots.transforms import pose_transform
from .spec import SPEC, TARGET_HALF_SIZE, TARGETS, surface


class UncapScene:
    root = "/World/Uncap"

    def __init__(self, backend, config):
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.prims import RigidPrim, SingleRigidPrim
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        self.backend, self.config = backend, config
        self.stage = backend.world.stage
        self.bodies = {}
        self.hard = PhysicsMaterial(
            self.root + "/Hard",
            static_friction=0.8,
            dynamic_friction=0.8,
            restitution=0,
        )
        self.sleeve = PhysicsMaterial(
            self.root + "/Sleeve",
            static_friction=SPEC.sleeve_friction,
            dynamic_friction=SPEC.sleeve_friction,
            restitution=0,
        )
        api = PhysxSchema.PhysxMaterialAPI.Apply(self.sleeve.material.GetPrim())
        api.CreateCompliantContactStiffnessAttr(SPEC.sleeve_stiffness)
        api.CreateCompliantContactDampingAttr(SPEC.sleeve_damping)
        api.CreateFrictionCombineModeAttr("min")
        for name, mass in (("body", 0.060), ("cap", 0.025)):
            path = self.root + "/" + name
            UsdGeom.Xform.Define(self.stage, path)
            UsdPhysics.RigidBodyAPI.Apply(self.stage.GetPrimAtPath(path))
            UsdPhysics.MassAPI.Apply(self.stage.GetPrimAtPath(path)).CreateMassAttr(
                mass
            )
            PhysxSchema.PhysxRigidBodyAPI.Apply(
                self.stage.GetPrimAtPath(path)
            ).CreateSolverPositionIterationCountAttr(32)
            self.bodies[name] = backend.world.scene.add(
                SingleRigidPrim(path, name="uncap_" + name)
            )
        self.shape(
            "body/Barrel",
            "cylinder",
            (SPEC.body_radius, SPEC.body_radius, SPEC.body_length),
            (0, 0, 0),
            (0.85, 0.65, 0.12),
        )
        # Polygonal closed cap. Circumferential overlap avoids cracks between wall boxes.
        inner, outer, segments = 0.016, SPEC.cap_radius, 16
        for i in range(segments):
            a = 2 * math.pi * i / segments
            self.shape(
                f"cap/Wall{i}",
                "box",
                (
                    outer - inner,
                    2 * outer * math.tan(math.pi / segments),
                    SPEC.cap_length,
                ),
                (
                    (outer + inner) / 2 * math.cos(a),
                    (outer + inner) / 2 * math.sin(a),
                    0,
                ),
                (0.18, 0.3, 0.85),
                angle=a,
            )
        self.shape(
            "cap/End",
            "cylinder",
            (outer, outer, 0.003),
            (0, 0, SPEC.cap_length / 2 - 0.0015),
            (0.18, 0.3, 0.85),
        )
        # Four tiny internal sleeve patches represent an elastic plastic friction fit.
        # Their interference produces normal force; Coulomb friction resists sliding.
        pad_thickness = 0.0012
        radial = SPEC.body_radius - SPEC.sleeve_interference + pad_thickness / 2
        for i in range(4):
            a = math.pi * i / 2
            self.shape(
                f"cap/Sleeve{i}",
                "box",
                (pad_thickness, 0.007, 0.014),
                (radial * math.cos(a), radial * math.sin(a), -0.021),
                (0.15, 0.25, 0.7),
                angle=a,
                compliant=True,
            )
        for name, xy in TARGETS.items():
            self.shape(
                f"Target_{name}",
                "box",
                (2 * TARGET_HALF_SIZE, 2 * TARGET_HALF_SIZE, 0.0004),
                (*xy, 0.0002),
                (0.6, 0.48, 0.1) if name == "body" else (0.12, 0.24, 0.6),
                collision=False,
            )
        paths = [self.root + "/body", self.root + "/cap"]
        self.fingers = {}
        for mount in config.robot.mounts:
            self.fingers[mount.name] = RigidPrim(
                mount.prim_path + "/link[78]",
                name="uncap_fingers_" + mount.name,
                reset_xform_properties=False,
                track_contact_forces=True,
                contact_filter_prim_paths_expr=paths,
                max_contact_count=1024,
                disable_stablization=False,
            )
        self.contacts = {
            name: RigidPrim(
                path,
                name="uncap_support_" + name,
                reset_xform_properties=False,
                track_contact_forces=True,
                contact_filter_prim_paths_expr=["/World/Bench", paths[1 - i]],
                max_contact_count=1024,
                disable_stablization=False,
            )
            for i, (name, path) in enumerate(zip(("body", "cap"), paths))
        }
        backend.planning_payloads = self.planning_payloads
        backend.scene_report["l2_01"] = {
            "spec": asdict(SPEC),
            "source": "project procedural asset; no downloaded mesh",
            "reference": "https://openai.robocurve.org/stationerybench/",
            "optional_visual_landmarks": TARGETS,
            "variant": "horizontal table pickup; both hands empty; local proxy, not original StationeryBench",
            "object_attachment": False,
            "separation": "passive compliant sleeve contact and friction",
            "calibration": "engineering parameters, not a measured commercial marker",
            "damage_model": "rigid shells; force threshold proxy only, no fracture simulation",
            "force_observation": "scorer only; GPT sees RGB-D and robot execution state",
            "grasp_check": "contact inspection; vertical lift guard inapplicable to a constrained cap",
        }

    def shape(
        self,
        name,
        kind,
        size,
        position,
        color,
        *,
        angle=0,
        compliant=False,
        collision=True,
    ):
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

        path = self.root + "/" + name
        obj = (
            UsdGeom.Cube.Define(self.stage, path)
            if kind == "box"
            else UsdGeom.Cylinder.Define(self.stage, path)
        )
        if kind == "box":
            obj.CreateSizeAttr(1)
        else:
            obj.CreateRadiusAttr(size[0])
            obj.CreateHeightAttr(size[2])
            obj.CreateAxisAttr("Z")
        x = UsdGeom.Xformable(obj)
        x.AddTranslateOp().Set(Gf.Vec3d(*position))
        x.AddRotateZOp().Set(math.degrees(angle))
        if kind == "box":
            x.AddScaleOp().Set(Gf.Vec3f(*size))
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            api = PhysxSchema.PhysxCollisionAPI.Apply(obj.GetPrim())
            api.CreateContactOffsetAttr(0.0005)
            api.CreateRestOffsetAttr(0)
            UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(
                (self.sleeve if compliant else self.hard).material,
                materialPurpose="physics",
            )

    def reset(self, seed):
        # Assembled marker lies on the table, pointing along world -Y.
        for name, body in self.bodies.items():
            body.set_default_state(
                position=np.array(
                    (
                        0.36,
                        0.045 - (SPEC.cap_offset if name == "cap" else 0),
                        SPEC.cap_radius + 0.001,
                    )
                ),
                orientation=np.array((math.sqrt(0.5), math.sqrt(0.5), 0, 0)),
                linear_velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
            )
        self.backend.reset(seed)
        for view in (*self.fingers.values(), *self.contacts.values()):
            view.initialize()

    def measure(self):
        dt = self.config.physics_dt_s
        fingers = {
            arm: np.asarray(v.get_contact_force_matrix(dt=dt))
            for arm, v in self.fingers.items()
        }
        if any(
            v.shape != (2, 2, 3) or not np.isfinite(v).all() for v in fingers.values()
        ):
            raise RuntimeError("Invalid uncapping finger contact data")
        rows = {}
        for i, (name, body) in enumerate(self.bodies.items()):
            p, q = body.get_world_pose()
            contact = np.asarray(self.contacts[name].get_contact_force_matrix(dt=dt))
            if contact.shape != (1, 2, 3) or not np.isfinite(contact).all():
                raise RuntimeError("Invalid marker support/pair contact data")
            rows[name] = {
                "pose": [*map(float, p), *map(float, q)],
                "linear_velocity": body.get_linear_velocity().tolist(),
                "angular_velocity": body.get_angular_velocity().tolist(),
                "fingers": {
                    a: np.linalg.norm(f[:, i], axis=1).tolist()
                    for a, f in fingers.items()
                },
                "bench_force_n": float(np.linalg.norm(contact[0, 0])),
                "pair_force_n": float(np.linalg.norm(contact[0, 1])),
            }
        return rows

    def planning_payloads(self):
        rows = self.measure()
        payloads = {"left": [], "right": []}
        for name, row in rows.items():
            t = pose_transform(row["pose"])
            for arm, forces in row["fingers"].items():
                if all(f > SPEC.contact_threshold for f in forces):
                    payloads[arm].append(
                        {
                            "surface_points_world": (
                                surface(name) @ t[:3, :3].T + t[:3, 3]
                            ).tolist(),
                            "source": "measured bilateral contact, collision only",
                        }
                    )
        return payloads
