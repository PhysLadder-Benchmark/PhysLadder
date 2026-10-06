"""Scoring geometry derived from the locked native colliders, never visual bounds.

Meshes are their authored convex hulls. Primitive surfaces use a fine polyhedral
approximation with a <=50 micrometre error budget deducted from contact tolerance.
Only collider volume below a receptacle's rim is checked. Source USDs stay intact.
"""

import json
from functools import lru_cache

import numpy as np

from ...paths import PROJECT_ROOT
from ...robots.transforms import pose_transform


@lru_cache(maxsize=1)
def catalog():
    return json.loads((PROJECT_ROOT / 'assets/tuojing/l1_scoring_geometry.json').read_text())['props']


def shapes(name):
    return catalog()[name]['shapes']


@lru_cache(maxsize=128)
def polyhedron(prop, index):
    shape = shapes(prop)[index]
    if shape['kind'] == 'mesh':
        points, edges = np.asarray(shape['points']), np.asarray(shape['edges'])
        error = 0.
    else:
        radius, height = shape['radius'], shape['height']
        angles = np.arange(64) * (2 * np.pi / 64)
        if shape['kind'] == 'cylinder':
            rings = [(radius, -height / 2), (radius, height / 2)]
        else:
            polar = np.linspace(-np.pi / 2, np.pi / 2, 33)
            rings = [(radius * np.cos(a), radius * np.sin(a) + sign * height / 2)
                     for sign, values in ((-1, polar[polar <= 0]), (1, polar[polar >= 0]))
                     for a in values]
        points = np.array([(r * np.cos(a), r * np.sin(a), z) for r, z in rings for a in angles])
        edges = [(j * 64 + i, j * 64 + (i + 1) % 64) for j in range(len(rings)) for i in range(64)]
        edges += [(j * 64 + i, (j + 1) * 64 + i) for j in range(len(rings)-1) for i in range(64)]
        # Triangular side diagonals are needed when clipping a tilted surface.
        edges += [(j * 64 + i, (j + 1) * 64 + (i + 1) % 64)
                  for j in range(len(rings)-1) for i in range(64)]
        edges = np.array(edges)
        if shape['axis'] == 'X': points = points[:, [2, 1, 0]]
        if shape['axis'] == 'Y': points = points[:, [0, 2, 1]]
        error = radius * (1 - np.cos(np.pi / 64)**2)
    transform = np.asarray(shape['transform'])
    return points @ transform[:3,:3].T + transform[:3,3], edges, error


def transformed_poly(prop, index, transform):
    vertices, edges, error = polyhedron(prop, index)
    return vertices @ transform[:3,:3].T + transform[:3,3], edges, error


def clip_below(vertices, edges, height):
    a, b = vertices[edges[:,0]], vertices[edges[:,1]]
    crossing = (a[:,2] < height) != (b[:,2] < height)
    a, b = a[crossing], b[crossing]
    cut = a + (b-a) * ((height-a[:,2]) / (b[:,2]-a[:,2]))[:,None]
    return np.concatenate((vertices[vertices[:,2] <= height], cut))


@lru_cache(maxsize=1)
def cup_cavity():
    planes = []
    for shape in shapes('ripple_paper_cup'):
        if not shape['name'].startswith('Wall_'): continue
        vertices = np.asarray(shape['points'])
        levels = np.unique(vertices[:,2])
        inner = []
        for z in levels:
            ring = vertices[np.isclose(vertices[:,2], z, atol=1e-9)]
            inner.extend(ring[np.argsort(np.linalg.norm(ring[:,:2],axis=1))[:2]])
        a,b,c = np.asarray(inner)[:3]
        normal = np.cross(b-a,c-a); normal /= np.linalg.norm(normal)
        if np.dot(normal[:2],a[:2]) < 0: normal = -normal
        planes.append([*normal, np.dot(normal,a)])
    floor_shape = next(i for i,s in enumerate(shapes('ripple_paper_cup')) if s['name']=='Floor')
    floor = polyhedron('ripple_paper_cup', floor_shape)[0][:,2].max()
    rim = max(max(np.asarray(s['points'])[:,2]) for s in shapes('ripple_paper_cup') if s['name'].startswith('Wall_'))
    return np.asarray(planes), float(floor), float(rim)


def cavity_fit(prop, transform, tolerance, *, indices=None):
    planes, floor, rim = cup_cavity()
    margins, entered = [], False
    for i in (range(len(shapes(prop))) if indices is None else indices):
        vertices, edges, error = transformed_poly(prop,i,transform)
        entered |= bool(vertices[:,2].min() < rim-tolerance)
        clipped = clip_below(vertices,edges,rim)
        if not len(clipped): continue
        margins.append(min(float(clipped[:,2].min()-floor),
                           float(np.min(planes[:,3] - clipped @ planes[:,:3].T))) - error)
    margin = min(margins,default=-1.)
    return bool(entered and margin >= -tolerance), margin


@lru_cache(maxsize=1)
def band_opening():
    """64 native inner edges; the common open band plane is z=35 mm."""
    planes=[]
    for shape in shapes('yellow_visor'):
        if not shape['name'].startswith('Band_'): continue
        points=np.asarray(shape['points']); ring=points[np.isclose(points[:,2],points[:,2].min(),atol=1e-9)]
        a,b=ring[np.argsort(np.linalg.norm(ring[:,:2],axis=1))[:2],:2]
        n=np.array([b[1]-a[1],a[0]-b[0]]); n/=np.linalg.norm(n)
        if n@a < 0:n=-n
        planes.append([*n,n@a])
    return np.asarray(planes)


def hook_geometry(rack_pose):
    shape=shapes('wooden_rack')[0]
    transform=pose_transform(rack_pose) @ np.asarray(shape['transform'])
    axis=transform[:3,2]; centre=transform[:3,3]
    return centre-axis*shape['height']/2, centre+axis*shape['height']/2, shape['radius']


def verify_stage(stage, roots):
    """Fail closed if an importer changes the geometry used by this scorer."""
    from pxr import UsdGeom
    cache=UsdGeom.XformCache()
    for prop, root in roots:
        parent=stage.GetPrimAtPath(root)
        inverse=np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(parent)).T)
        for shape in shapes(prop):
            prim=stage.GetPrimAtPath(root+'/Colliders/'+shape['name'])
            if not prim or prim.GetTypeName().lower()!=shape['kind']:
                raise ValueError('Native scoring collider mismatch: '+str(prim.GetPath()) if prim else root+'/'+shape['name'])
            transform=inverse @ np.asarray(cache.GetLocalToWorldTransform(prim)).T
            if not np.allclose(transform,shape['transform'],atol=1e-7,rtol=0):
                raise ValueError('Native scoring transform mismatch: '+str(prim.GetPath()))
            if shape['kind']=='mesh':
                points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                if not all(np.min(np.linalg.norm(points-v,axis=1)) < 1e-7 for v in shape['points']):
                    raise ValueError('Native scoring vertices mismatch: '+str(prim.GetPath()))
            else:
                schema=UsdGeom.Cylinder(prim) if shape['kind']=='cylinder' else UsdGeom.Capsule(prim)
                if (schema.GetAxisAttr().Get()!=shape['axis'] or
                    not np.allclose([schema.GetRadiusAttr().Get(),schema.GetHeightAttr().Get()],
                                    [shape['radius'],shape['height']],atol=1e-9,rtol=0)):
                    raise ValueError('Native scoring primitive mismatch: '+str(prim.GetPath()))
