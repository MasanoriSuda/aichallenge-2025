"""Score recorded CV peer predictions with future GNSS only as held-out truth.

Post-intervention observations cannot validate an unobserved no-braking future.
The ego stopping path is counterfactual after Emergency changes publication.
"""
from pathlib import Path
import bisect
import json
import math
import yaml

run = Path('output/20260909-failure-bundle-dev2-r1')
replay = Path('output/20260909-failure-bundle-execution-replay')
source = next((run/'d2/mpcc_architecture_snapshots').glob('000000001629*/snapshot.yaml'))
world = yaml.safe_load(source.read_text())['source']['replay_world']
report = yaml.safe_load((replay/'report.yaml').read_text())
peer = world['obstacles'][0]
assert peer['id'] == 'd1'
motion = json.loads((run/'d1-longitudinal-full-motion.json').read_text())['rows']
rows = sorted(motion['/sensing/gnss/pose_with_covariance'])
times = [row[0] for row in rows]

def position(t):
    j = bisect.bisect_right(times, t)
    assert 0 < j < len(rows)
    a, b = rows[j-1:j+1]
    assert 0 < b[0]-a[0] < .2
    f = (t-a[0])/(b[0]-a[0])
    return [a[k]+f*(b[k]-a[k]) for k in (2,3)]

def clearance(pose, center):
    # Same circle/axis-aligned asymmetric footprint distance as native geometry.
    f = world['physical_footprint']
    dx, dy = center[0]-pose['x_m'], center[1]-pose['y_m']
    c, s = math.cos(pose['yaw_rad']), math.sin(pose['yaw_rad'])
    x, y = c*dx+s*dy, -s*dx+c*dy
    left, right = f['left_extent_m']+f['margin_m'], f['right_extent_m']+f['margin_m']
    front, rear = f['front_extent_m']+f['margin_m'], f['rear_extent_m']+f['margin_m']
    outside = math.hypot(max(-rear-x,0,x-front), max(-right-y,0,y-left))
    inside = min(x+rear,front-x,y+right,left-y) if -rear <= x <= front and -right <= y <= left else 0
    return outside-inside-peer['radius_m']

t0 = world['observed_sec']
observations = []
for dt in [0,.25,.5,.7,1.,1.5,1.555,1.63]:
    cv = [peer['x_m']+peer['velocity_x_mps']*dt,peer['y_m']+peer['velocity_y_mps']*dt]
    truth = position(t0+dt)
    observations.append(dict(elapsed_sec=dt,cv=cv,held_out_gnss=truth,error_m=math.dist(cv,truth)))
traces = []
for trace in report['terminal_traces']:
    dt = trace['minimum_elapsed_sec']; pose = trace['minimum_pose']
    cv = [peer['x_m']+peer['velocity_x_mps']*dt,peer['y_m']+peer['velocity_y_mps']*dt]
    calculated = clearance(pose,cv)
    assert abs(calculated-trace['minimum_clearance_m']) < 1e-8
    truth = position(t0+dt)
    traces.append(dict(normal_path_reference=trace['normal_path_reference'],elapsed_sec=dt,
        native_clearance_m=trace['minimum_clearance_m'],python_clearance_m=calculated,
        held_out_gnss_clearance_m=clearance(pose,truth),
        held_out_full_path_minimum_clearance_m=min(clearance(p,position(t0+p['elapsed_sec'])) for p in trace['full_path']),
        authority=False))
output = dict(observation_sec=t0,peer=peer,observations=observations,traces=traces,
    limitation='Hindsight is scoring only. D1 later enters moving Emergency at wall 1788903989.013246009 (~0.73s after D2), so later GNSS includes intervention. Neither future braking nor any hypothetical unobserved peer path may grant current authority. No predictor root cause established.')
(replay/'held-out-peer-score.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(output,indent=2))
