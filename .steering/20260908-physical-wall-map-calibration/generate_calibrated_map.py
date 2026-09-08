"""Diagnostic additive projection of near-vertical static collision faces.

Calibrated additive coverage; limits: all heights are projected, and faces with abs(normal.z)
above0.5 are omitted. This tests map feasibility/coverage, not a claim that
every possible body contact is represented by the selected face class.
"""
import json
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

root = Path('output/20260908-physical-wall-map-calibration')
map_root = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3')
config = yaml.safe_load((map_root / 'occupancy_grid_map.yaml').read_text())
baseline = root / 'original-occupancy-grid.pgm'
import hashlib
assert hashlib.sha256(baseline.read_bytes()).hexdigest() == 'c24af2130a8df96047a49d5b7759af7a0b29645c023e7f3bc6d4ba436275725a'
original = np.array(Image.open(baseline))
result = original.copy()
height, width = original.shape
origin = np.array(config['origin'][:2])
resolution = config['resolution']
assert config['origin'][2] == 0 and config['negate'] == 0
mesh = np.load('output/20260908-mpcc-stop-reference-single-r3/unity-track-collision.npz')
triangles = mesh['vertices_mgrs'][mesh['triangles']]
normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
lengths = np.linalg.norm(normals, axis=1)
selected = np.flatnonzero((lengths > 1e-8) & (abs(normals[:, 2]) <= .5*lengths))
covered = []
for triangle_id in selected:
    triangle = (triangles[triangle_id, :, :2]-origin)/resolution
    # Native RecoveryFootprint uses origin as cell(0,0)'s center, not its corner.
    lower = np.maximum(0, np.ceil(triangle.min(axis=0)-.5).astype(int))
    upper = np.minimum([width-1, height-1], np.floor(triangle.max(axis=0)+.5).astype(int))
    if np.any(lower > upper):
        continue
    xx, yy = np.meshgrid(np.arange(lower[0], upper[0]+1), np.arange(lower[1], upper[1]+1))
    centers = np.c_[xx.ravel(), yy.ravel()].astype(float)
    # SAT: cell axes plus every projected triangle edge normal. Degenerate
    # line projections remain covered; boundary-touching cells are occupied.
    edges = np.roll(triangle, -1, axis=0)-triangle
    axes = np.r_[np.eye(2), np.c_[-edges[:, 1], edges[:, 0]]]
    overlaps = np.ones(len(centers), dtype=bool)
    for axis in axes:
        if np.linalg.norm(axis) < 1e-12:
            continue
        projected = triangle @ axis
        center = centers @ axis
        support = .5*np.abs(axis).sum()
        overlaps &= (center+support >= projected.min()) & (center-support <= projected.max())
    result[height-1-yy.ravel()[overlaps], xx.ravel()[overlaps]] = 0
    if np.any(overlaps):
        covered.append(int(triangle_id))
Image.fromarray(result).save(root / 'diagnostic-wall-candidate.pgm')
report = dict(method=__doc__, selected_faces=len(selected), projected_faces=len(covered),
              changed_cells=int(np.count_nonzero(result != original)),
              newly_occupied_free_cells=int(np.count_nonzero((original == 255) & (result == 0))),
              original_occupied_preserved=bool(np.all(result[original == 0] == 0)),
              contact_faces_covered=all(i in covered for i in [81055, 81056]),
              projection_scope='All heights; abs(normal.z)<=0.5; additive union with original map')
(root / 'diagnostic-wall-candidate.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
