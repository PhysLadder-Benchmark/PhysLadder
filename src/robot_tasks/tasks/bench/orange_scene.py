"""Native GPU FEM orange; no rigid proxy, attachments or rupture scene swapping."""

import numpy as np

from ...asset_delivery import verified_prop
from ...types import InvalidSimulationState
from ..l1.scene import L1Scene
from .orange_evaluation import deformation_metrics


class OrangeScene:
    root = "/World/L2"
    set_pose = L1Scene.set_pose
    add_part = L1Scene.add_part

    def __init__(self, backend, config, spec):
        from pxr import PhysxSchema, UsdGeom

        if not backend.settings.gpu_dynamics:
            raise ValueError(
                "L2-06 native orange requires simulation.gpu_dynamics=true"
            )
        self.backend, self.config, self.spec, self.design = (
            backend,
            config,
            spec,
            spec.design,
        )
        self.stage = backend.world.stage
        self.colliders = []
        self.path = self.root + "/orange"
        asset, evidence = verified_prop("orange_segment")
        prim = self.stage.DefinePrim(self.path)
        prim.GetReferences().AddReference(str(asset))
        self.placement = self.design.articles[0].initial
        self.set_pose(self.path, self.placement)
        for part in self.design.fixtures:
            self.add_part(self.root + "/Fixture/" + part.name, part)
        self.visual = self.stage.GetPrimAtPath(self.path + "/Visual")
        self.sim = self.stage.GetPrimAtPath(self.path + "/Simulation")
        self.collision = self.stage.GetPrimAtPath(self.path + "/Collision")
        # Explicit task contact margins for the small FEM asset in the full
        # metre-scale laboratory. Material, rest mesh and mass are unchanged.
        contact = PhysxSchema.PhysxCollisionAPI.Apply(self.collision)
        contact.CreateContactOffsetAttr(0.003)
        contact.CreateRestOffsetAttr(0.0015)
        self.rest = np.asarray(
            self.sim.GetAttribute("omniphysics:restShapePoints").Get(), dtype=float
        )
        self.tets = np.asarray(
            self.sim.GetAttribute("omniphysics:restTetVtxIndices").Get(), dtype=int
        )
        r = self.rest[self.tets]
        edges = np.stack([r[:, i] - r[:, 0] for i in (1, 2, 3)], axis=2)
        self.rest_inverse = np.linalg.inv(edges)
        self.initial = {"orange": self.placement}
        from .surface_distance import link_triangles

        self.finger_paths, self.finger_surfaces = [], []
        bounds = UsdGeom.BBoxCache(0.0, ["default", "render"])
        for arm in ("left", "right"):
            mount = next(m for m in config.robot.mounts if m.name == arm)
            for link in ("link7", "link8"):
                path = mount.prim_path + "/" + link
                self.finger_paths.append(path)
                self.finger_surfaces.append(link_triangles(self.stage, path))
        self.previous = self.cached = None
        self.observer_bounds = {}
        for arm in config.observation_arms:
            mount = next(m for m in config.robot.mounts if m.name == arm)
            parts = []
            for index in range(1, 9):
                path = mount.prim_path + f"/link{index}"
                b = bounds.ComputeUntransformedBound(
                    self.stage.GetPrimAtPath(path)
                ).ComputeAlignedRange()
                parts.append((path, np.asarray(b.GetMin()), np.asarray(b.GetMax())))
            self.observer_bounds[arm] = parts
        backend.planning_fixture_paths = ()
        backend.planning_payloads = self.planning_payloads
        backend.scene_report["household"] = {
            "version": self.design.version,
            "mechanism": spec.mechanism,
            "assets": [evidence],
            "limitations": spec.limitations,
            "parameters": spec.parameters,
            "initial_poses": self.initial,
            "native_materials_preserved": True,
            "object_attachment": False,
            "contact_overrides_m": {"contact_offset": 0.003, "rest_offset": 0.0015},
            "rupture_demo_executed": False,
            "soft_contact_measurement": "FEM collision vertices to delivered finger triangle surfaces, distances capped at 10 mm; unsigned proximity, not force or grasp proof",
            "deformation": "95th-percentile principal element strain, rigid-motion invariant; engineering reference only",
        }

    def reset(self, seed):
        # A new episode constructs a fresh stage. No state-switch demo or
        # mid-episode vertex rewrite is used to recover from manipulation.
        self.backend.reset(seed)
        self.previous = self.cached = None

    def measure(self):
        from omni.physx import get_physx_interface
        from pxr import UsdGeom

        now = self.backend.time_s()
        if self.cached is not None and self.previous[0] == now:
            return self.cached
        get_physx_interface().update_transformations(False, True, True, False)
        points = np.asarray(self.visual.GetAttribute("points").Get(), dtype=float)
        simulation = np.asarray(self.sim.GetAttribute("points").Get(), dtype=float)
        collision = np.asarray(self.collision.GetAttribute("points").Get(), dtype=float)
        cache = UsdGeom.XformCache()
        t = np.asarray(cache.GetLocalToWorldTransform(self.visual)).T
        world = points @ t[:3, :3].T + t[:3, 3]
        ct = np.asarray(cache.GetLocalToWorldTransform(self.collision)).T
        physical = collision @ ct[:3, :3].T + ct[:3, 3]
        if (
            world.shape[0] < 4
            or physical.shape[0] < 4
            or not np.isfinite(world).all()
            or not np.isfinite(simulation).all()
            or not np.isfinite(physical).all()
        ):
            raise InvalidSimulationState("Missing/nonfinite native FEM readback")
        speed = (
            0.0
            if self.previous is None
            else float(
                np.linalg.norm(world - self.previous[1], axis=1).max()
                / (now - self.previous[0])
            )
        )
        distances = []
        from .surface_distance import surface_clearance
        for path, triangles in zip(self.finger_paths, self.finger_surfaces):
            finger = np.asarray(
                cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))
            ).T
            local = (physical - finger[:3, 3]) @ finger[:3, :3]
            distances.append(surface_clearance(local, triangles))
        row = {
            "bounds_min": world.min(axis=0).tolist(),
            "bounds_max": world.max(axis=0).tolist(),
            "collision_bounds_min": physical.min(axis=0).tolist(),
            "collision_bounds_max": physical.max(axis=0).tolist(),
            "visual_world_transform": t.tolist(),
            "centroid": world.mean(axis=0).tolist(),
            "speed_m_s": speed,
            "finger_distances_m": distances,
            **deformation_metrics(self.rest_inverse, simulation, self.tets),
        }
        row["observer_proximity"] = {}
        for arm, parts in self.observer_bounds.items():
            nearest = float("inf")
            for path, lo, hi in parts:
                transform = np.asarray(
                    cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))
                ).T
                local = (physical - transform[:3, 3]) @ transform[:3, :3]
                distance = np.linalg.norm(
                    np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1
                ).min()
                nearest = min(nearest, float(distance))
            row["observer_proximity"][arm] = {
                "mode": "soft_geometry_proximity",
                "distance_m": nearest,
                "scope": "soft collision surface to observer link bounds; contact ambiguous",
            }
        self.previous, self.cached = (now, world.copy()), {"orange": row}
        return self.cached

    def observer_evidence(self):
        return self.measure()["orange"]["observer_proximity"]

    def planning_payloads(self):
        row = self.measure()["orange"]
        payloads = {a: [] for a in self.config.arms}
        if (all(d < 0.003 for d in row["finger_distances_m"][:2])
                and row["bounds_min"][2] > self.spec.parameters["table_z"] + .01):
            from itertools import product

            payloads["left"].append(
                {
                    "surface_points_world": list(
                        product(*zip(row["bounds_min"], row["bounds_max"]))
                    ),
                    "source": "measured FEM bounds near fingers; conservative collision proxy, not force evidence",
                }
            )
        return payloads
