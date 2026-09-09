"""Audit cached WheelHit direction against discrete body/steering epochs.

Private recorded states diagnose the simulator component only. This is not a
deployable predictor or a fitted actuator delay. The inspected execution order
sets the tire, reads GetGroundHit, then applies forces at its cached contact.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path('output/20260910-shared-plant-contact-epoch-r1')


def stats(values):
    a = np.asarray(values)
    return dict(count=len(a), mae=float(abs(a).mean()),
                p95=float(np.percentile(abs(a), 95)), maximum=float(abs(a).max()))


def evaluate(name, identity, begin, end):
    source = Path(f'output/20260909-force-step-analysis-{name}-r1/physics.npz')
    raw = np.load(source)
    ix = {str(k): i for i, k in enumerate(raw['columns'])}
    a = raw['values']; a = a[a[:, ix['id']] == identity]
    rot = Rotation.from_quat(a[:, [ix['eq_'+k] for k in 'xyzw']])
    tire = np.radians(a[:, ix['steer_deg']])
    mask = (a[:, ix['fixed']] >= begin) & (a[:, ix['fixed']] < end)
    mask &= a[:, ix['vehicle_speed']] >= .5
    mask[:2] = False
    wheels = {}
    for n in range(4):
        key = f'w{n}_'
        actual = a[:, [ix[key+'f_'+k] for k in 'xyz']]
        normal = a[:, [ix[key+'n_'+k] for k in 'xyz']]
        good = mask & (a[:, ix[key+'ground']] == 1)
        arms = {}
        # Body epoch and tire epoch are independent, not a single lag to fit.
        for body_back in [0, 1, 2]:
            for tire_back in ([0, 1, 2] if n < 2 else [0]):
                angle = np.roll(tire, tire_back) if n < 2 else np.zeros(len(a))
                body_forward = np.stack([np.sin(angle), np.zeros(len(a)), np.cos(angle)], axis=1)
                world_forward = Rotation.from_quat(np.roll(rot.as_quat(), body_back, axis=0)).apply(body_forward)
                projected = world_forward - np.sum(world_forward*normal, axis=1)[:, None]*normal
                length = np.linalg.norm(projected, axis=1)
                projected /= np.maximum(length, 1e-30)[:, None]
                error = np.arctan2(np.linalg.norm(np.cross(projected, actual), axis=1),
                                   np.sum(projected*actual, axis=1))
                arms[f'body_minus_{body_back}_tire_minus_{tire_back}'] = stats(error[good])
        wheels[str(n)] = arms
    return dict(source=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                body_id=identity, window_sec=[begin, end], wheels=wheels)


OUT.mkdir(exist_ok=False)
report = dict(authority=False, meaning=__doc__,
              single=evaluate('single', -823441338, 16, 59.54),
              dev2={d: evaluate('dev2', i, 7.585, 9.709999782)
                    for d, i in [('d1', 1594692888), ('d2', -615963298)]})
(OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
for name, result in [('single', report['single']), *report['dev2'].items()]:
    for wheel, arms in result['wheels'].items():
        print(name, wheel, arms, flush=True)
