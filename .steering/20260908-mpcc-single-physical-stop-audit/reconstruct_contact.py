"""Diagnostic reconstruction from frozen sensors and unmodified Unity assets.

Uses the serialized convex body mesh, not a fitted collision margin. Triangle
clipping detects geometric overlap; this is not a PhysX contact-event replay.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

root = Path('output/20260908-mpcc-stop-reference-single-r3')
sensors = json.loads((root / 'first-failure-raw-sensors.json').read_text())
motion = json.loads((root / 'first-failure-motion.json').read_text())['rows']
geometry = json.loads((root / 'unity-collider-geometry.json').read_text())
body = np.array(next(c for c in geometry['colliders']
                     if c['path'] == '/GoKart1/Colliders/Collider')['vertices_base_link_xyz'])
hull = ConvexHull(body)
scene = np.load(root / 'unity-track-collision.npz')
triangles = scene['vertices_mgrs'][scene['triangles']]
indices = np.flatnonzero(np.all(triangles.max(axis=1) >= [89663, 43163, 41], axis=1)
                        & np.all(triangles.min(axis=1) <= [89673, 43174, 44], axis=1))
triangles = triangles[indices]


def nearest(rows, stamp):
    row = min(rows, key=lambda r: abs(r[0] - stamp))
    assert abs(row[0] - stamp) < .001, (stamp, row[0])
    return row


def clip(poly, normal, offset):
    result = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        da, db = a @ normal + offset, b @ normal + offset
        if da <= 0:
            result.append(a)
        if (da < 0 < db) or (db < 0 < da):
            result.append(a + da / (da - db) * (b - a))
    return np.array(result)


results = []
for gnss in sensors['/sensing/gnss/nav_sat_fix']:
    stamp = gnss[0]
    if not 250.3 < stamp < 250.76:
        continue
    imu = nearest(sensors['/sensing/imu/imu_raw'], stamp)
    pose = nearest(motion['/sensing/gnss/pose_with_covariance'], stamp)
    # Actual Unity imu_link local yaw -90 becomes ROS yaw +90. Its publisher
    # converts transform.rotation, so undo this local rotation on the right.
    rotation = Rotation.from_quat(imu[3:7]) * Rotation.from_euler('z', -np.pi / 2)
    # Undo GNSSPoser's planar lever arm, then use the actual rigid orientation.
    antenna = np.array([pose[2] - .26 * np.cos(pose[7]),
                        pose[3] - .26 * np.sin(pose[7]), gnss[5]])
    origin = antenna + rotation.apply([.26, 0, 0])
    local_triangles = (triangles - origin) @ rotation.as_matrix()
    overlap = []
    for triangle_id, triangle in zip(indices, local_triangles):
        if np.any(triangle.max(axis=0) < body.min(axis=0)) or np.any(
                triangle.min(axis=0) > body.max(axis=0)):
            continue
        polygon = triangle.copy()
        for plane in hull.equations:
            polygon = clip(polygon, plane[:3], plane[3])
            if not len(polygon):
                break
        if len(polygon):
            overlap.append(dict(triangle_id=int(triangle_id),
                                clipped_polygon_base_link=polygon.tolist(),
                                triangle_base_link=triangle.tolist()))
    results.append(dict(source=stamp, base_link_mgrs=origin.tolist(),
                        base_link_rpy=rotation.as_euler('xyz').tolist(),
                        body_mgrs_bounds=[(rotation.apply(body)+origin).min(axis=0).tolist(),
                                          (rotation.apply(body)+origin).max(axis=0).tolist()],
                        intersecting_triangles=overlap))
    print(f'{stamp:.6f} pose={origin} rpy={rotation.as_euler("xyz")} overlaps={len(overlap)}')
(root / 'reconstructed-body-wall-intersections.json').write_text(
    json.dumps(dict(method=__doc__, body_hull_facets=len(hull.equations),
                    scene_region_triangles=len(triangles), samples=results), indent=2) + '\n')
