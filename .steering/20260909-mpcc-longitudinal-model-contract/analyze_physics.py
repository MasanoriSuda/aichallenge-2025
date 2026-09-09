"""Summarize one-time runtime observations; no parameter fitting or authority."""
from pathlib import Path
import hashlib
import json
import re
import numpy as np

run = Path('output/20260909-longitudinal-physics-observed-dev2-r1')
out = Path('output/20260909-longitudinal-physics-analysis-r1')
out.mkdir(exist_ok=False)
log = (run / 'unity-player.log').read_text(errors='replace')
assert 'MPCC_INPUT_OBS error' not in log
records = {}
for line in log.splitlines():
    if not line.startswith('MPCC_PHYSICS_OBS '):
        continue
    row = dict(re.findall(r'(\w+)=([^ ]+)', line))
    identity = int(row['id'])
    records.setdefault(identity, []).append(row)
assert len(records) == 2

def vector(row, prefix):
    return np.array([float(row[prefix + axis]) for axis in 'xyz'])

result = []
for identity, rows in sorted(records.items()):
    body = [row for row in rows if 'wheel' not in row]
    wheels = [row for row in rows if 'wheel' in row]
    assert len(body) == 1 and len(wheels) == 4
    body = body[0]
    assert {int(row['wheel']) for row in wheels} == set(range(4))
    assert all(row['source_ns'] == body['source_ns'] for row in wheels)
    xyz = vector(body, 'rotation_')
    q = np.r_[xyz, float(body['rotation_w'])]
    q /= np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    origin = vector(body, 'body_')
    com = vector(body, 'com_local_')
    actual_com = vector(body, 'com_world_')
    com_error = float(np.linalg.norm(origin + rotation @ com - actual_com))
    wheel_data = []
    for wheel in sorted(wheels, key=lambda row: int(row['wheel'])):
        local = rotation.T @ (vector(wheel, 'position_') - origin)
        wheel_data.append(dict(index=int(wheel['wheel']),
            sprung_mass_kg=float(wheel['sprung_mass']),
            grounded=wheel['grounded'] == 'True',
            steer_deg=float(wheel['steer_deg']),
            local_unity_xyz_m=local.tolist(),
            local_com_unity_xyz_m=(local-com).tolist()))
    mass = float(body['mass'])
    front_mass = sum(row['sprung_mass_kg'] for row in wheel_data[:2])
    grounded_front_mass = sum(row['sprung_mass_kg'] for row in wheel_data[:2] if row['grounded'])
    result.append(dict(receiver_id=identity, source_ns=int(body['source_ns']),
        observed_body=body, wheels=wheel_data,
        total_sprung_mass_kg=sum(row['sprung_mass_kg'] for row in wheel_data),
        front_sprung_mass_fraction=front_mass/mass,
        grounded_front_sprung_mass_fraction=grounded_front_mass/mass,
        front_cancellation_coefficient_per_sec=float(body['skid'])*front_mass/mass/.005,
        grounded_front_cancellation_coefficient_per_sec=float(body['skid'])*grounded_front_mass/mass/.005,
        reconstructed_world_com_error_m=com_error))

report = dict(authority=False, run=str(run),
    log_sha256=hashlib.sha256((run/'unity-player.log').read_bytes()).hexdigest(),
    vehicle_config_warnings=[line for line in log.splitlines() if 'physics.gripSteerFactor=' in line],
    receivers=result,
    limitations=[
        'Only two moving samples; receiver identities are not assigned to ROS Domains in this report.',
        'WheelCollider sprungMass is its configured supported mass, not a measured instantaneous contact force.',
        'Grounded flags and wheel positions are the last physics update as seen at input application; no full-time contact history.',
        'Coefficient calculation uses local 5ms physics step and is not a whole-vehicle identification or uncertainty bound.',
        'Continuous all-grounded planar fit omits contact changes, suspension/roll/pitch and engine forces.',
        'YAML grip .6 is clamped by local runtime to .7. Existing controller steering gain is not changed.',
    ])
assert len(report['vehicle_config_warnings']) == 2
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
