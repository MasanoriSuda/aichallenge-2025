"""Read-only extraction of declared Unity collider vertices in base_link axes."""
import hashlib
import json
import math
from pathlib import Path

import UnityPy
from UnityPy.helpers.MeshHelper import MeshHandler

asset_root = Path('aichallenge/simulator/AWSIM/AWSIM_Data')
env = UnityPy.load(str(asset_root / 'level1'))
objects = {obj.path_id: obj for obj in env.objects}
resources = {}
transforms = {o.path_id: o.parse_as_dict() for o in objects.values() if o.type.name == 'Transform'}

def transform(point, t):
    x, y, z = [point[i] * t['m_LocalScale'][axis] for i, axis in enumerate('xyz')]
    q = t['m_LocalRotation']
    qx, qy, qz, qw = [q[axis] for axis in 'xyzw']
    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    tx, ty, tz = 2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x)
    r = [x+qw*tx+qy*tz-qz*ty, y+qw*ty+qz*tx-qx*tz, z+qw*tz+qx*ty-qy*tx]
    return [r[i] + t['m_LocalPosition'][axis] for i, axis in enumerate('xyz')]

def kart_point(point, tid):
    while tid != kart_root:
        t = transforms[tid]
        point = transform(point, t)
        tid = t['m_Father']['m_PathID']
        assert tid, 'Collider is not a GoKart1 descendant'
    return point

kart_root = 718
base = kart_point([0, 0, 0], 796)
assert transforms[796]['m_LocalRotation'] == dict(x=0., y=0., z=0., w=1.)

def base_point(point, tid):
    p = [a-b for a, b in zip(kart_point(point, tid), base)]
    # Unity forward/right/up -> ROS forward/left/up.
    return [p[2], -p[0], p[1]]

out = []

def walk(tid, parent, active):
    t = transforms[tid]
    g = objects[t['m_GameObject']['m_PathID']].parse_as_dict()
    active = active and g['m_IsActive']
    path = parent + '/' + g['m_Name']
    for c in g['m_Component']:
        obj = objects[c['component']['m_PathID']]
        if obj.type.name != 'MeshCollider':
            continue
        d = obj.parse_as_dict()
        mesh_pointer = d['m_Mesh']
        file_id, path_id = mesh_pointer['m_FileID'], mesh_pointer['m_PathID']
        file_name = 'level1' if file_id == 0 else obj.assets_file.externals[file_id-1].path
        if file_name not in resources:
            loaded = UnityPy.load(str(asset_root / file_name))
            resources[file_name] = {o.path_id: o for o in loaded.objects}
        mesh = resources[file_name][path_id].parse_as_object()
        handler = MeshHandler(mesh)
        handler.process()
        vertices = [base_point(p, tid) for p in handler.m_Vertices]
        bounds = [[min(v[i] for v in vertices), max(v[i] for v in vertices)] for i in range(3)]
        out.append(dict(path=path, active_in_asset=active, enabled=d['m_Enabled'],
                        trigger=d['m_IsTrigger'], convex=d['m_Convex'],
                        mesh_file=file_name, mesh_id=path_id, mesh_name=mesh.m_Name,
                        bounds_base_link_xyz=bounds, vertices_base_link_xyz=vertices))
    for c in t['m_Children']:
        walk(c['m_PathID'], path, active)


result = []
for kart_root in [718, 616, 526, 686]:
    descendants = {}
    def locate(tid, parent):
        t = transforms[tid]
        g = objects[t['m_GameObject']['m_PathID']].parse_as_dict()
        path = parent + '/' + g['m_Name']
        descendants[path] = tid
        for c in t['m_Children']:
            locate(c['m_PathID'], path)
    locate(kart_root, '')
    kart_name = next(p for p in descendants if p.count('/') == 1)
    base_id = descendants[kart_name + '/URDF/base_link']
    sensor_id = descendants[kart_name + '/URDF/base_link/gnss_link/NavsatfixSensor']
    base = kart_point([0, 0, 0], base_id)
    assert transforms[base_id]['m_LocalRotation'] == dict(x=0., y=0., z=0., w=1.)
    antenna = base_point([0, 0, 0], sensor_id)
    out = []
    walk(kart_root, '', True)
    body = next(c for c in out if c['path'] == kart_name + '/Colliders/Collider')
    relative = [[v[i]-antenna[i] for i in range(3)] for v in body['vertices_base_link_xyz']]
    extreme = max(relative, key=lambda p: sum(x*x for x in p))
    result.append(dict(vehicle=kart_name, sensor_transform=sensor_id, antenna_base_link=antenna,
                       body_mesh_file=body['mesh_file'],body_mesh_id=body['mesh_id'],
                       body_active=body['active_in_asset'],body_enabled=body['enabled'],
                       body_trigger=body['trigger'],body_convex=body['convex'],
                       bounds_base_link_xyz=body['bounds_base_link_xyz'],
                       nominal_3d_radius_m=math.sqrt(sum(x*x for x in extreme)),
                       extreme_body_point_relative_to_antenna=extreme))
Path('output/20260908-mpcc-stop-reference-single-r3/unity-all-kart-peer-envelope.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
