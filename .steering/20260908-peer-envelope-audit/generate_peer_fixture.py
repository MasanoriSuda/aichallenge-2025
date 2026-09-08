"""Reduce the independently extracted solid mesh to its exact 3D convex hull."""
from pathlib import Path
import hashlib
import json
import math
from scipy.spatial import ConvexHull

root = Path('output/20260908-mpcc-stop-reference-single-r3')
source = root/'unity-collider-geometry.json'
envelopes = root/'unity-all-kart-peer-envelope.json'
body = next(c for c in json.loads(source.read_text())['colliders']
            if c['path'] == '/GoKart1/Colliders/Collider')
vertices = body['vertices_base_link_xyz']
indices = sorted(ConvexHull(vertices).vertices.tolist())
vehicles = json.loads(envelopes.read_text())
assert len(vehicles) == 4
for vehicle in vehicles:
    assert vehicle['body_mesh_file'] == body['mesh_file']
    assert vehicle['body_mesh_id'] == body['mesh_id']
    assert vehicle['bounds_base_link_xyz'] == body['bounds_base_link_xyz']
    antenna = vehicle['antenna_base_link']
    radius = max(math.dist(p, antenna) for p in vertices)
    assert abs(radius - vehicle['nominal_3d_radius_m']) < 1e-12
    assert abs(max(math.dist(vertices[i], antenna) for i in indices)-radius) < 1e-12
projection = json.loads(Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/fixtures/awsim_body_projection.json').read_text())
fixture = dict(calibration_id='local-awsim-gnss-peer-body-20260909', frame='base_link', units='m',
               v2x_position_frame='gnss_antenna', asset_sha256=projection['asset_sha256'],
               source_extraction='.steering/20260908-peer-envelope-audit/extract_peer_envelopes.py',
               hull_generator='.steering/20260908-peer-envelope-audit/generate_peer_fixture.py',
               source_files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,envelopes]},
               vehicles=vehicles, solid_mesh_vertex_count=len(vertices),
               convex_hull_vertices_base_link_xyz_m=[vertices[i] for i in indices])
destination = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/fixtures/awsim_peer_body_envelope.json')
destination.write_text(json.dumps(fixture,indent=2)+'\n')
print(len(vertices),'mesh vertices ->',len(indices),'hull vertices; all4antenna origins checked')
