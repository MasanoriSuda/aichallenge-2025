"""One-step cached contact frame, with causal public-input rollouts.

The previous contact point/direction is reconstructed from past predicted body
and tire states, not future contact truth. Physical timestep is the measured
5ms simulator setting, not a newly fitted receiver or actuator delay. Contact
weights remain frozen original training values. Initial previous frame is a
constant-body-twist backward extrapolation, explicitly not an observation.
"""
from pathlib import Path
import bisect
import importlib.util
import json
import math
import numpy as np

spec = importlib.util.spec_from_file_location('corrected', Path(__file__).with_name('compare_base_link_models.py'))
corrected = importlib.util.module_from_spec(spec); spec.loader.exec_module(corrected)
base = corrected.base
ORIGINAL = base.integrate
DT = base.DT
OUT = Path('output/20260910-shared-plant-cached-contact-r1')


def rotate(yaw, xy):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*xy[0]-s*xy[1], s*xy[0]+c*xy[1]])


def force_derivative(state, wire, previous, previous_tire):
    x, y, yaw, u, vy, r, unused = state
    com = state[:2]+rotate(yaw, [base.P['com_x_m'], 0])
    previous_com = previous[:2]+rotate(previous[2], [base.P['com_x_m'], 0])
    acceleration = max(wire, -u/.01) if u < 0 else wire
    rolling = -math.copysign(min(.37, abs(u)/.01), u) if u else 0
    fx = fy = torque = 0.
    for n, geometry in enumerate(base.GEOMETRY):
        point = previous_com+rotate(previous[2], geometry)
        wx, wy = rotate(-yaw, point-com)
        angle = previous[2]-yaw+(previous_tire if n < 2 else 0)
        c, s = math.cos(angle), math.sin(angle)
        side = -s*(u-r*wy)+c*(vy+r*wx)
        lateral = -base.CORNER[n]*side
        drive = base.TRACTION[n]*(acceleration+rolling)
        ax, ay = c*drive-s*lateral, s*drive+c*lateral
        fx += ax; fy += ay; torque += wx*ay-wy*ax
    ref_vy = vy-base.P['com_x_m']*r
    return np.array([u*math.cos(yaw)-ref_vy*math.sin(yaw),
                     u*math.sin(yaw)+ref_vy*math.cos(yaw), r,
                     fx-.03*u+r*vy, fy-.03*vy-r*u,
                     torque*base.P['mass_kg']/base.P['yaw_inertia_kgm2']-r, 0.])


def integrate(initial, tire_initial, history, future, duration, model):
    if model != 'dynamic_yaw':
        return ORIGINAL(initial, tire_initial, history, future, duration, model)
    commands = history+future
    stamps = [row[0] for row in commands]
    assert stamps == sorted(stamps)
    def command(t):
        i = bisect.bisect_right(stamps, t)-1
        assert i >= 0
        return commands[i][1:]
    state = initial.copy(); previous = initial.copy(); tire = tire_initial
    previous[2] -= initial[5]*DT
    previous[:2] -= rotate(initial[2], [initial[3], initial[4]-base.P['com_x_m']*initial[5]])*DT
    previous_tire = tire_initial
    predictions = []
    for k in range(round(duration/DT)):
        elapsed = k*DT
        wire, unused = command(elapsed)
        unused, delayed = command(elapsed-.1)
        demand = .7*np.clip(delayed, -math.pi/6, math.pi/6)
        tire += np.clip(DT/(.02+DT)*(demand-tire), -math.radians(80)*DT, math.radians(80)*DT)
        a = force_derivative(state, wire, previous, previous_tire)
        next_state = state+DT*force_derivative(state+.5*DT*a, wire, previous, previous_tire)
        previous = state.copy(); previous_tire = tire
        state = next_state
        if k+1 in [20, 50, 100, 200]:
            predictions.append((k+1, state.copy(), tire))
    return predictions


base.integrate = integrate
if __name__ == '__main__':
    OUT.mkdir(exist_ok=False)
    report = dict(authority=False, meaning=__doc__, parameters=base.P,
                  single_holdout=corrected.evaluate('single', 'd1', -823441338, 35, 59.54),
                  dev2={d: corrected.evaluate('dev2', d, i, 7.585, 9.709999782)
                        for d, i in [('d1', 1594692888), ('d2', -615963298)]})
    (OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    for name, result in [('single', report['single_holdout']), *report['dev2'].items()]:
        print(name, result['score']['dynamic_yaw'], flush=True)
