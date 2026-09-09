"""Compare odometry with calibrated raw IMU/velocity in each recorded run.

Linear interpolation between source stamps is for scoring only. The new runs
have no private Rigidbody probe; calibrated raw IMU is a reference observation,
not independent physical truth or an exact replay of the original controller.
"""
from pathlib import Path
import json
import math
import numpy as np


def yaw(q):
    n = math.sqrt(sum(v*v for v in q.values()))
    x, y, z, w = [q[k]/n for k in 'xyzw']
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def body_yaw(q):
    # q_world_imu * q_imu_body; measured body->imu is +pi/2.
    x, y, z, w = [q[k] for k in 'xyzw']; c = math.sqrt(.5)
    return yaw(dict(x=c*(x-y), y=c*(x+y), z=c*(z-w), w=c*(w+z)))


def score(values):
    a = np.asarray(values)
    return dict(count=len(a), mean=float(a.mean()), mae=float(abs(a).mean()),
                p95=float(np.percentile(abs(a), 95)), maximum=float(abs(a).max()))


def compare(path, end):
    data = json.loads(Path(path).read_text())
    imu = sorted([r for r in data if r['topic'].endswith('imu_raw')], key=lambda r: r['stamp'])
    speed = sorted([r for r in data if r['topic'].endswith('velocity_status')], key=lambda r: r['stamp'])
    ti = np.array([r['stamp'] for r in imu]); yi = np.unwrap([body_yaw(r['orientation']) for r in imu])
    tv = np.array([r['stamp'] for r in speed]); vx = np.array([r['v'] for r in speed])
    rows = []
    for r in data:
        if not r['topic'].endswith('kinematic_state') or r['v'] < .5 or r['stamp'] >= end:
            continue
        t = r['stamp']
        if not max(ti[0], tv[0]) <= t <= min(ti[-1], tv[-1]):
            continue
        reference_yaw = np.interp(t, ti, yi)
        error = yaw(r['orientation'])-reference_yaw
        rows.append(dict(stamp=t, heading_error_rad=math.atan2(math.sin(error), math.cos(error)),
                         velocity_error_mps=r['v']-float(np.interp(t, tv, vx))))
    return dict(path=str(path), end_sec=end, heading=score([r['heading_error_rad'] for r in rows]),
                velocity=score([r['velocity_error_mps'] for r in rows]), rows=rows)


out = Path('output/20260909-measured-heading-state-comparison-r1'); out.mkdir(exist_ok=False)
report = dict(authority=False, limits=__doc__, results={})
for name, path, end in [
    ('old-d1', 'output/20260909-force-step-ros-state-dev2-r1/d1.json', 9.709999782),
    ('old-d2', 'output/20260909-force-step-ros-state-dev2-r1/d2.json', 9.709999782),
    ('new-d1', 'output/20260909-measured-heading-ros-dev2-r1/d1.json', 10.309999769),
    ('new-d2', 'output/20260909-measured-heading-ros-dev2-r1/d2.json', 10.309999769),
]:
    result = compare(path, end); report['results'][name] = result
    print(name, {k: v for k, v in result.items() if k != 'rows'})
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
