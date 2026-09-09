"""Compare same-source-epoch ROS state with diagnostic Rigidbody observations.

Future truth is used only for scoring. Timestamp alignment is not a claim that
the controller received the sample by that source time. No fitted map transform.
"""
from pathlib import Path
import json
import numpy as np


def score(x):
    x = np.asarray(x)
    return dict(count=len(x), mean=float(x.mean()), mae=float(abs(x).mean()),
                p95=float(np.percentile(abs(x), 95)), maximum=float(abs(x).max())) if len(x) else dict(count=0)


def wrap(x):
    return np.arctan2(np.sin(x), np.cos(x))


def evaluate(name, domain, identity, begin, end):
    raw = np.load(f'output/20260909-force-step-analysis-{name}-r1/physics.npz')
    ix = {str(k): i for i, k in enumerate(raw['columns'])}
    a = raw['values']; a = a[a[:, ix['id']] == identity]
    time = a[:, ix['fixed']]
    x, y, z, w = a[:, [ix['eq_'+k] for k in 'xyzw']].T
    rotation = np.stack([1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                         2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                         2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], axis=1).reshape(-1, 3, 3)
    body_velocity = np.einsum('nji,nj->ni', rotation, a[:, [ix['ev_'+k] for k in 'xyz']])
    body_omega = np.einsum('nji,nj->ni', rotation, a[:, [ix['ew_'+k] for k in 'xyz']])
    truth = dict(v=body_velocity[:, 2], vy=-body_velocity[:, 0], yaw_rate=-body_omega[:, 1],
                 steering=-a[:, ix['steer_deg']]*np.pi/180,
                 yaw=wrap(np.arctan2(rotation[:, 2, 2], rotation[:, 0, 2])-np.pi/2))
    rows = json.loads(Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json').read_text())
    result = {}
    for suffix in ['velocity_status', 'steering_status', 'imu_raw', 'kinematic_state']:
        errors = {}
        for row in rows:
            if not row['topic'].endswith(suffix) or not begin <= row['stamp'] < end:
                continue
            j = min(np.searchsorted(time, row['stamp']), len(time)-1)
            if j and abs(time[j-1]-row['stamp']) < abs(time[j]-row['stamp']):
                j -= 1
            if abs(time[j]-row['stamp']) > 1e-7 or truth['v'][j] < .5:
                continue
            observed = {k: row[k] for k in ['v', 'vy', 'yaw_rate', 'steering'] if k in row}
            if suffix == 'imu_raw':
                observed['yaw_rate'] = row['angular']['z']
            if suffix == 'kinematic_state':
                q = row['orientation']
                observed['yaw'] = np.arctan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
            for key, value in observed.items():
                error = value-truth[key][j]
                if key == 'yaw':
                    error = wrap(error)
                # Preserve Euler-wrap outliers; do not hide them as valid gyro.
                errors.setdefault(key, []).append(error)
        result[suffix] = {k: score(v) for k, v in errors.items()}
    return dict(window_sec=[begin, end], results=result)


out = Path('output/20260909-force-step-ros-state-comparison-r1'); out.mkdir(exist_ok=False)
report = dict(authority=False, limitation=__doc__,
              single=evaluate('single', 'd1', -823441338, 16, 59.54),
              dev2={d: evaluate('dev2', d, i, 7.585, 9.709999782)
                    for d, i in [('d1', 1594692888), ('d2', -615963298)]})
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
