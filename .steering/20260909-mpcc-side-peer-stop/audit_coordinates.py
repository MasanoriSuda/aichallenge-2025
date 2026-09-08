"""Independent common-frame check of the saved native obstacle support inputs."""
from pathlib import Path
import json
import math
import numpy as np
import yaml

root = Path('output/20260909-side-peer-stop-audit')
path = next((root/'zero/mpcc_architecture_snapshots').glob('*/snapshot.yaml'))
data = yaml.load(path.read_text(),Loader=yaml.CSafeLoader)
source = data['source']
world = source['replay_world']
peer = world['obstacles'][0]
request = source['semantic_request']
knots = source['wall_course_frame_knots']
progress = np.array([k['progress_m'] for k in knots])
primal = np.array(data['warm_start']['primal'])
foot = world['physical_footprint']

def frame(theta):
    s = source['course_progress_origin_m']+theta
    i = min(len(knots)-2,max(0,int(np.searchsorted(progress,s)-1)))
    a,b = knots[i:i+2]
    t = (s-a['progress_m'])/(b['progress_m']-a['progress_m'])
    dh = math.remainder(b['heading_rad']-a['heading_rad'],2*math.pi)
    return np.array([a['x_m']+t*(b['x_m']-a['x_m']),a['y_m']+t*(b['y_m']-a['y_m'])]),a['heading_rad']+t*dh

def rot(h):
    return np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])

def clearance(relative,heading):
    p = rot(heading).T@relative
    q = np.clip(p,[-foot['rear_extent_m']-foot['margin_m'],-foot['right_extent_m']-foot['margin_m']],
                [foot['front_extent_m']+foot['margin_m'],foot['left_extent_m']+foot['margin_m']])
    return float(np.linalg.norm(p-q)-peer['radius_m'])

rows = []
elapsed = source['control_prediction_origin_sec']-world['observed_sec']
for i,(stage,prediction) in enumerate(zip(request['inputs'],source['dynamic_obstacle_stages'])):
    elapsed += stage['stage_dt_sec']
    ey,lag,epsi,v,theta,delta,rho = primal[(i+1)*7:(i+2)*7]
    origin,h = frame(theta)
    peer_world = np.array([peer['x_m']+elapsed*peer['velocity_x_mps'],peer['y_m']+elapsed*peer['velocity_y_mps']])
    # Exact peer displacement in the ego reference's axes at this same stage.
    local_peer = rot(h).T@(peer_world-origin)
    true_relative = local_peer-np.array([lag,ey])
    old_relative = np.array([prediction['target_progress_m']-theta-lag,prediction['target_lateral_m']-ey])
    rows.append(dict(stage=i+1,elapsed_sec=elapsed,
                     old_relative=old_relative.tolist(),common_frame_relative=true_relative.tolist(),
                     displacement_error_m=float(np.linalg.norm(old_relative-true_relative)),
                     old_clearance_m=clearance(old_relative,epsi),
                     common_frame_clearance_m=clearance(true_relative,epsi)))
report=dict(scope='same saved affine witness; coordinate check, not a certified control rollout',
            source=str(path),maximum_displacement_error_m=max(r['displacement_error_m'] for r in rows),rows=rows)
(root/'coordinate-audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
for row in rows:
    if row['old_clearance_m']*row['common_frame_clearance_m'] < 0:
        print(json.dumps(row))
