"""Collision-only simulator geometry. Never inserted into a policy observation.

The planner is privileged to the table/fixture meshes and robot configuration.
A task may register physically measured held-body surface samples for clearance
checking; this does not attach the object in physics or establish task success.
"""

import numpy as np


def collider_meshes(backend):
    from pxr import Usd, UsdGeom, UsdPhysics

    from ..backends.scene_geometry import BENCH_POSITION, BENCH_SIZE

    result = {
        "cuboid": {
            "bench": {"pose": [*BENCH_POSITION, 1, 0, 0, 0], "dims": list(BENCH_SIZE)}
        },
        "mesh": {},
    }
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    fixture_paths = set(getattr(backend, "planning_fixture_paths", ()))
    for prim in backend.world.stage.Traverse():
        path = str(prim.GetPath())
        if path not in fixture_paths or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        transform = np.array(cache.GetLocalToWorldTransform(prim)).T
        if prim.IsA(UsdGeom.Mesh):
            shape = UsdGeom.Mesh(prim)
            vertices = np.array(shape.GetPointsAttr().Get(), dtype=float)
            indices = list(shape.GetFaceVertexIndicesAttr().Get())
            faces, cursor = [], 0
            for count in shape.GetFaceVertexCountsAttr().Get():
                face = indices[cursor : cursor + count]
                cursor += count
                faces.extend(
                    [face[0], face[i], face[i + 1]] for i in range(1, count - 1)
                )
        elif prim.IsA(UsdGeom.Cube):
            size = UsdGeom.Cube(prim).GetSizeAttr().Get()
            vertices = (
                np.array(
                    [
                        (x, y, z)
                        for x in (-0.5, 0.5)
                        for y in (-0.5, 0.5)
                        for z in (-0.5, 0.5)
                    ]
                )
                * size
            )
            faces = [
                [0, 1, 3],
                [0, 3, 2],
                [4, 6, 7],
                [4, 7, 5],
                [0, 4, 5],
                [0, 5, 1],
                [2, 3, 7],
                [2, 7, 6],
                [0, 2, 6],
                [0, 6, 4],
                [1, 5, 7],
                [1, 7, 3],
            ]
        elif prim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(prim)
            radius, height = shape.GetRadiusAttr().Get(), shape.GetHeightAttr().Get()
            angles = np.arange(64) * 2 * np.pi / 64
            vertices = np.array(
                [
                    (radius * np.cos(a), radius * np.sin(a), z)
                    for z in (-height / 2, height / 2)
                    for a in angles
                ]
            )
            faces = []
            for i in range(64):
                j = (i + 1) % 64
                faces.extend([[i, j, 64 + j], [i, 64 + j, 64 + i]])
            faces += [[0, i + 1, i] for i in range(1, 63)] + [
                [64, 64 + i, 65 + i] for i in range(1, 63)
            ]
            axis = shape.GetAxisAttr().Get()
            if axis == "X":
                vertices = vertices[:, [2, 1, 0]]
            if axis == "Y":
                vertices = vertices[:, [0, 2, 1]]
        else:
            raise ValueError(f"Unsupported planning fixture collider: {path}")
        vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
        result["mesh"][path.replace("/", "_")] = {
            "pose": [0, 0, 0, 1, 0, 0, 0],
            "vertices": vertices.tolist(),
            "faces": faces,
        }
    return result


def snapshot(backend):
    if not hasattr(backend, "_planner_static_scene"):
        backend._planner_static_scene = collider_meshes(backend)
    callback = getattr(backend, "planning_payloads", None)
    return {
        "obstacles": backend._planner_static_scene,
        "payloads": callback() if callback is not None else {},
        "information_source": "privileged simulator collision geometry; never task targets or score",
        "limitations": "unheld movable objects excluded for contact; approximate R5 collision spheres",
    }


def box_surface(size, resolution=5):
    axes = [np.linspace(-s / 2, s / 2, resolution) for s in size]
    return np.array(
        [
            (x, y, z)
            for x in axes[0]
            for y in axes[1]
            for z in axes[2]
            if abs(abs(x) - size[0] / 2) < 1e-8
            or abs(abs(y) - size[1] / 2) < 1e-8
            or abs(abs(z) - size[2] / 2) < 1e-8
        ]
    )


def article_surface(article):
    from ..robots.transforms import origin_transform

    if article.asset:
        # Conservative box for imported objects, disclosed as such in planner logs.
        return box_surface(article.asset_size)
    points = []
    for part in article.parts:
        if not part.collision:
            continue
        if part.shape == "box":
            local = box_surface(part.size)
        elif part.shape == "mesh":
            triangles = np.asarray(part.vertices)[np.asarray(part.faces)]
            weights = np.array([(i / 4, j / 4, 1 - (i + j) / 4)
                                for i in range(5) for j in range(5 - i)])
            local = np.einsum("wk,fkd->fwd", weights, triangles).reshape(-1, 3)
        elif part.shape == "sphere":
            azimuth = np.linspace(0, 2 * np.pi, 32, endpoint=False)
            polar = np.linspace(0, np.pi, 17)
            local = np.array([(part.size[0] * np.sin(p) * np.cos(a),
                               part.size[0] * np.sin(p) * np.sin(a), part.size[0] * np.cos(p))
                              for p in polar for a in azimuth])
        else:
            radius, _, height = part.size
            angles = np.linspace(0, 2 * np.pi, 48, endpoint=False)
            local = np.array(
                [
                    (r * np.cos(a), r * np.sin(a), z)
                    for z in np.linspace(-height / 2, height / 2, 7)
                    for r in (0, radius / 2, radius)
                    for a in angles
                    if r == radius or abs(abs(z) - height / 2) < 1e-8
                ]
            )
        t = origin_transform(part.position, part.rotation)
        points.extend(local @ t[:3, :3].T + t[:3, 3])
    return np.asarray(points)
