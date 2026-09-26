"""Small PhysX mechanism checks, without a robot policy or model API.

Run in native Windows Isaac Sim 5.1 Python. Every moving object is dynamic;
the script applies gravity/forces, never overwrites poses during a rollout.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("kind", choices=["insertion", "friction", "spring"])
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--dt", type=float, default=1 / 240)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
started = time.time()

from isaacsim import SimulationApp
from runtime_config import simulation_config

app = SimulationApp(simulation_config())

import numpy as np
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
from PIL import Image

world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=1 / 60)
stage = world.stage
world.get_physics_context().set_solver_type("TGS")
UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900)


def material(path, friction):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    return mat


def collider(prim, mat=None, enabled=True):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(enabled)
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(0.0001)
    api.CreateRestOffsetAttr(0.0)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics"
        )


def dynamic(prim, mass):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    p = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    p.CreateSolverPositionIterationCountAttr(16)
    p.CreateSolverVelocityIterationCountAttr(4)
    p.CreateEnableCCDAttr(True)


def box(path, pos, size, mat=None, mass=None, color=(0.35, 0.5, 0.7)):
    geom = UsdGeom.Cube.Define(stage, path)
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    collider(geom.GetPrim(), mat)
    if mass is not None:
        dynamic(geom.GetPrim(), mass)
    return geom.GetPrim()


cases = []
if args.kind == "insertion":
    mat = material("/World/Material", 0.3)
    box("/World/Floor", (0, 0, -0.01), (2, 2, 0.02), mat)
    for i, (gap, offset, collision) in enumerate([
        (0.0005, 0, True), (0.0005, 0.03, True),
        (0.003, 0, True), (0.003, 0.03, True),
        (0.0005, 0.03, False),
    ]):
        x = (i - 2) * 0.2
        root = f"/World/Case_{i}"
        inner, outer, height = 0.01 + gap, 0.085, 0.08
        # A ring of separate convex wedges retains the hole. One convex hull
        # of the whole ring would incorrectly seal it.
        for j in range(32):
            a, b = 2 * math.pi * j / 32, 2 * math.pi * (j + 1) / 32
            points = [(x + r * math.cos(t), r * math.sin(t), z)
                      for z in (0, height) for r, t in
                      [(inner, a), (outer, a), (outer, b), (inner, b)]]
            mesh = UsdGeom.Mesh.Define(stage, f"{root}/Wall_{j}")
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr([4] * 6)
            mesh.CreateFaceVertexIndicesAttr([
                3, 2, 1, 0, 4, 5, 6, 7, 0, 1, 5, 4,
                1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7,
            ])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.5, 0.7)])
            collider(mesh.GetPrim(), mat, collision)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        geom = UsdGeom.Cylinder.Define(stage, f"{root}/Body")
        geom.CreateRadiusAttr(0.01)
        geom.CreateHeightAttr(0.04)
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(x + offset, 0, 0.14))
        geom.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.25, 0.15)])
        collider(geom.GetPrim(), mat)
        dynamic(geom.GetPrim(), 0.1)
        cases.append({"id": i, "gap_m": gap, "offset_m": offset,
                      "wall_collision": collision, "origin": [x, 0, 0.14]})
    camera_position, camera_target = (0.7, -1.05, 0.95), (0, 0, 0.04)
    duration = 2.0
elif args.kind == "friction":
    for i, (mu, mass) in enumerate([(0.05, 1.0), (0.8, 1.0), (0.05, 4.0)]):
        y = (i - 1) * 0.35
        mat = material(f"/World/Mat_{i}", mu)
        box(f"/World/Case_{i}/Floor", (1, y, -0.025), (4, 0.3, 0.05), mat)
        box(f"/World/Case_{i}/Body", (0, y, 0.04), (0.08, 0.08, 0.08), mat, mass,
            color=(0.85, 0.25 + i * 0.15, 0.15))
        cases.append({"id": i, "friction": mu, "mass_kg": mass,
                      "force_x_N": 1.0, "origin": [0, y, 0.04]})
    camera_position, camera_target = (2.3, -2.2, 2.4), (0.65, 0, 0)
    duration = 2.5
else:
    for i, k in enumerate([100.0, 1000.0]):
        x, z, mass = (i - 0.5) * 0.2, 0.2, 0.1
        root = f"/World/Case_{i}"
        body = box(f"{root}/Body", (x, 0, z), (0.06, 0.06, 0.02), mass=mass,
                   color=(0.85, 0.25, 0.15))
        PhysxSchema.PhysxRigidBodyAPI.Apply(body).CreateDisableGravityAttr(True)
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/SpringJoint")
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/Body")])
        joint.CreateLocalPos0Attr(Gf.Vec3f(x, 0, z))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-0.05)
        joint.CreateUpperLimitAttr(0.05)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(k)
        drive.CreateDampingAttr(2 * math.sqrt(k * mass))
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(1000.0)
        box(f"{root}/Base", (x, 0, 0.12), (0.08, 0.08, 0.02))
        cases.append({"id": i, "stiffness_N_per_m": k, "force_z_N": -2.0,
                      "expected_compression_m": 2.0 / k, "origin": [x, 0, z]})
    camera_position, camera_target = (0.4, -0.65, 0.55), (0, 0, 0.15)
    duration = 2.5

view = world.scene.add(RigidPrim(
    "/World/Case_.*/Body", name="bodies", track_contact_forces=True,
    prepare_contact_sensors=True, max_contact_count=256,
))
camera = rep.create.camera(position=camera_position, look_at=camera_target, clipping_range=(0.005, 100))
product = rep.create.render_product(camera, (1024, 768))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(product)
rep.orchestrator.set_capture_on_play(False)
world.reset()
stage.GetRootLayer().Export(str(args.output / "initial_scene.usda"))
rows, frames = [], []
total_steps = round(duration / args.dt)
body_paths = list(view.prim_paths)
# Resolve path order explicitly rather than assuming lexicographic PhysX order.
case_ids = [int(path.split("Case_")[1].split("/")[0]) for path in body_paths]
for step in range(total_steps):
    t = step * args.dt
    forces = np.zeros((len(cases), 3), dtype=np.float32)
    if args.kind == "friction" and t >= 0.5:
        forces[:, 0] = 1.0
    if args.kind == "spring" and 0.25 <= t < 1.5:
        forces[:, 2] = -2.0
    view.apply_forces(forces)
    render = step % max(1, round(1 / (20 * args.dt))) == 0
    world.step(render=False)
    positions, _ = view.get_world_poses()
    velocities = view.get_linear_velocities()
    contacts = view.get_net_contact_forces(dt=args.dt)
    for index, case_id in enumerate(case_ids):
        rows.append({"step": step, "time_s": (step + 1) * args.dt, "case_id": case_id,
                     **dict(zip(["x", "y", "z"], map(float, positions[index]))),
                     **dict(zip(["vx", "vy", "vz"], map(float, velocities[index]))),
                     **dict(zip(["contact_fx", "contact_fy", "contact_fz"], map(float, contacts[index]))),
                     "applied_fx": float(forces[index, 0]), "applied_fz": float(forces[index, 2])})
    if render:
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=2)
        pixels = rgb.get_data()
        if pixels is not None and np.asarray(pixels).size:
            frames.append(Image.fromarray(np.asarray(pixels)[..., :3].astype(np.uint8)))

with (args.output / "trajectory.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
checks = []
for case in cases:
    rr = [r for r in rows if r["case_id"] == case["id"]]
    tail = rr[-max(1, round(0.2 / args.dt)):]
    final = {key: float(np.mean([r[key] for r in tail])) for key in ["x", "y", "z", "contact_fz"]}
    if args.kind == "insertion":
        expect_blocked = case["offset_m"] > 0 and case["wall_collision"]
        passed = final["z"] > 0.075 if expect_blocked else final["z"] < 0.035
        details = {"expected_blocked": expect_blocked, "final": final}
    elif args.kind == "friction":
        displacement = final["x"] - case["origin"][0]
        expect_slide = case["force_x_N"] > case["friction"] * case["mass_kg"] * 9.81
        passed = displacement > 0.5 if expect_slide else abs(displacement) < 0.03
        # An incorrectly scaled contact sensor must fail the check too.
        force_ok = abs(final["contact_fz"] - case["mass_kg"] * 9.81) < case["mass_kg"] * 9.81 * 0.15
        passed = passed and force_ok
        details = {"expected_slide": expect_slide, "displacement_m": displacement,
                   "contact_readout_ok": force_ok, "final": final}
    else:
        loaded = [r["z"] for r in rr if 1.0 <= r["time_s"] < 1.45]
        compression = case["origin"][2] - float(np.mean(loaded))
        residual = case["origin"][2] - final["z"]
        expected = case["expected_compression_m"]
        passed = abs(compression - expected) < expected * 0.1 and abs(residual) < 0.0002
        details = {"compression_m": compression, "unloaded_residual_m": residual, "final": final}
    checks.append({**case, **details, "passed": bool(passed)})

if frames:
    frames[0].save(args.output / "first_frame.png")
    frames[-1].save(args.output / "last_frame.png")
    frames[0].save(args.output / "rollout.gif", save_all=True, append_images=frames[1:], duration=50, loop=0)
summary = {"kind": args.kind, "physics_dt": args.dt, "duration_s": duration,
           "platform": platform.platform(), "body_paths": body_paths,
           "wall_time_s": time.time() - started, "rendered_frames": len(frames),
           "checks": checks, "physics_passed": all(c["passed"] for c in checks),
           "passed": all(c["passed"] for c in checks) and len(frames) > 5,
           "scope": "object-level mechanism check; no robot, no model, no real-physics calibration"}
(args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print("MECHANISM_RESULT " + json.dumps(summary), flush=True)
app.close()
raise SystemExit(0 if summary["passed"] else 2)
