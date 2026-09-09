"""Replay the locally inspected Unity actuator with prescribed applied inputs.

This validates the post-receiver plant, not DDS arrival or availability of future
inputs to a controller. Only the first tire state is taken from measurements.
"""
from pathlib import Path
import json
import numpy as np


def score(values):
    values = np.abs(np.asarray(values))
    return dict(count=len(values), mae=float(values.mean()),
                p95=float(np.percentile(values, 95)), maximum=float(values.max()))


def evaluate(path, identity, end):
    raw = np.load(path)
    ix = {str(k): i for i, k in enumerate(raw['columns'])}
    a = raw['values']
    a = a[(a[:, ix['id']] == identity) & (a[:, ix['fixed']] < end)]
    f = np.float32
    state = previous = f(a[0, ix['steer_deg']])
    queue = []
    rows = []
    for row in a:
        time = f(row[ix['fixed']])
        dt = f(row[ix['dt']])
        demand = f(f(row[ix['steer_input_deg']]) * f(.7))
        queue.append((time, demand))
        while queue and f(time - queue[0][0]) > f(f(.1) + f(.1)):
            queue.pop(0)
        cutoff = f(time - f(.1))
        delayed = state
        for stamp, value in queue:
            if stamp > cutoff:
                break
            delayed = value
        alpha = f(dt / f(f(.02) + dt))
        target = f(state + f(alpha * f(delayed - state)))
        change = f(target - previous)
        limit = f(f(80) * dt)
        state = f(previous + f(np.clip(change, -limit, limit)))
        previous = state
        # Queue content before the recording is unknown; exclude initialization.
        if row[ix['fixed']] - a[0, ix['fixed']] >= .3:
            rows.append(dict(time=float(time), actual_deg=float(row[ix['steer_deg']]),
                             predicted_deg=float(state), error_deg=float(state-row[ix['steer_deg']])))
    return dict(steps=len(a), evaluated=score([x['error_deg'] for x in rows]), rows=rows)


out = Path('output/20260909-force-step-steering-actuator-r1')
out.mkdir(exist_ok=False)
report = dict(authority=False, limitation=__doc__, initialization_exclusion_sec=.3,
              single=evaluate('output/20260909-force-step-analysis-single-r1/physics.npz', -823441338, 59.54),
              dev2={str(i): evaluate('output/20260909-force-step-analysis-dev2-r1/physics.npz', i, 24.3)
                    for i in [1594692888, -615963298]})
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(dict(single=report['single']['evaluated'],
                     dev2={k: v['evaluated'] for k, v in report['dev2'].items()}), indent=2))
