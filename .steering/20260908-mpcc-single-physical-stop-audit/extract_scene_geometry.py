"""Extract the actual static collision mesh for diagnostic coordinate comparison."""
import json
from pathlib import Path

import numpy as np
import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

root = Path('aichallenge/simulator/AWSIM/AWSIM_Data')
environment = UnityPy.load(str(root / 'level1'))
objects = {o.path_id: o for o in environment.objects}

def matrix(t):
    q = np.array([t['m_LocalRotation'][a] for a in 'xyzw'])
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                               [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                               [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result[:3, :3] *= [t['m_LocalScale'][a] for a in 'xyz']
    result[:3, 3] = [t['m_LocalPosition'][a] for a in 'xyz']
    return result

transform = np.eye(4)
tid = 793
while tid:
    t = objects[tid].parse_as_dict()
    transform = matrix(t) @ transform
    tid = t['m_Father']['m_PathID']
assets = UnityPy.load(str(root / 'sharedassets1.assets'))
mesh = next(o for o in assets.objects if o.path_id == 108).parse_as_object()
handler = MeshHandler(mesh)
handler.process()
vertices = np.array(handler.m_Vertices)
world = vertices @ transform[:3, :3].T + transform[:3, 3]
# Environment MonoBehaviour1298: actual serialized mgrsOffsetPosition.
mgrs = np.c_[world[:, 2]+89637.703125, -world[:, 0]+43503.5, world[:, 1]+35.400001525878906]
triangles = np.array([t for submesh in handler.get_triangles() for t in submesh], dtype=np.int32)
destination = Path('output/20260908-mpcc-stop-reference-single-r3')
np.savez_compressed(destination / 'unity-track-collision.npz', vertices_unity_world=world,
                    vertices_mgrs=mgrs, triangles=triangles)
print(json.dumps(dict(mesh=mesh.m_Name, vertices=len(world), triangles=len(triangles),
                      mgrs_bounds=[mgrs.min(axis=0).tolist(), mgrs.max(axis=0).tolist()],
                      unity_bounds=[world.min(axis=0).tolist(), world.max(axis=0).tolist()])))
