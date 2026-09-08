"""Offline frame comparison on the recorded controls; no solve or promotion.

A: actual native exact-physical adapter output.
B: covariant arc-length Frenet equations, still with each stored stage curvature.
C: independent Cartesian body ODE, with identical actuator parameters/controls.
D: covariant coordinates using derivatives of the actual piecewise course frame.
"""
from pathlib import Path
import bisect
import hashlib
import json
import math
import sys

import numpy as np
from scipy.integrate import solve_ivp
import yaml

run = Path(sys.argv[1])
source = next((run/'d1/mpcc_architecture_snapshots').glob('000000006782*/snapshot.yaml'))
data = yaml.safe_load(source.read_text())
e = data['execution_evidence']
native_path = run/'executed-replay/6782-exact-physical.yaml'
native = yaml.safe_load(native_path.read_text())['points']
knots = data['source']['wall_course_frame_knots']
progress_knots = [k['progress_m'] for k in knots]
origin = e['course_progress_origin_m']
initial = e['predicted_states'][0]
wheelbase, gain, tau = (e[k] for k in ['wheelbase_m', 'yaw_response_gain', 'yaw_response_time_constant_sec'])


def frame(progress):
    if not progress_knots[0]-1e-5 <= progress <= progress_knots[-1]+1e-5:
        raise ValueError(('outside recorded geometry', progress))
    i = min(max(0, bisect.bisect_right(progress_knots, progress)-1), len(knots)-2)
    low, high = knots[i:i+2]
    ds = high['progress_m'] - low['progress_m']
    ratio = min(1., max(0., (progress-low['progress_m'])/ds))
    dh = math.atan2(math.sin(high['heading_rad']-low['heading_rad']), math.cos(high['heading_rad']-low['heading_rad']))
    heading = low['heading_rad'] + ratio * dh
    dx = (high['x_m']-low['x_m'])/ds
    dy = (high['y_m']-low['y_m'])/ds
    return (low['x_m']+ratio*ds*dx, low['y_m']+ratio*ds*dy,
            heading, dx, dy, dh/ds)


def reconstruct(state):
    ey, lag, epsi, velocity, progress, delta, rho = state
    x, y, h, *_ = frame(origin+progress)
    return np.array([x+lag*math.cos(h)-ey*math.sin(h),
                     y+lag*math.sin(h)+ey*math.cos(h), h+epsi])


state_keys = ['lateral_m', 'lag_m', 'heading_offset_rad', 'velocity_mps',
              'progress_m', 'steering_rad', 'response_steering_rad']
f0 = np.array([initial[k] for k in state_keys])
pose0 = reconstruct(f0)
cart0 = np.array([0., 0., pose0[2], f0[3], f0[5], f0[6]])


def cartesian_rhs(control):
    acceleration, rate = control['acceleration_mps2'], control['steering_rate_radps']

    def rhs(time, state):
        x, y, yaw, speed, delta, rho = state
        return [speed*math.cos(yaw), speed*math.sin(yaw),
                gain*speed*math.tan(rho)/wheelbase, acceleration, rate, (delta-rho)/tau]
    return rhs


def frenet_rhs(control, actual_frame):
    acceleration, rate, nu = (control[k] for k in ['acceleration_mps2', 'steering_rate_radps', 'virtual_progress_speed_mps'])

    def rhs(time, state):
        ey, lag, epsi, speed, progress, delta, rho = state
        curvature = control['path_curvature_radpm']
        tangent_derivative, normal_derivative = 1., 0.
        if actual_frame:
            _, _, h, dx, dy, curvature = frame(origin+progress)
            tangent_derivative = math.cos(h)*dx + math.sin(h)*dy
            normal_derivative = -math.sin(h)*dx + math.cos(h)*dy
        return [speed*math.sin(epsi) - (normal_derivative+curvature*lag)*nu,
                speed*math.cos(epsi) - (tangent_derivative-curvature*ey)*nu,
                gain*speed*math.tan(rho)/wheelbase-curvature*nu,
                acceleration, nu, rate, (delta-rho)/tau]
    return rhs


states = {'B': f0.copy(), 'C': cart0.copy(), 'D': f0.copy()}
rows = []
start = 0.
for index, control in enumerate(e['control_stages']):
    end = start + control['duration_sec']
    functions = {'B': frenet_rhs(control, False), 'C': cartesian_rhs(control),
                 'D': frenet_rhs(control, True)}
    solutions = {}
    for name, rhs in functions.items():
        sol = solve_ivp(rhs, [start, end], states[name], method='DOP853',
                       atol=1e-11, rtol=1e-11, max_step=0.001, dense_output=True)
        if not sol.success:
            raise RuntimeError(sol.message)
        solutions[name] = sol.sol
        states[name] = sol.y[:, -1]
    for point in native:
        time = point['elapsed_sec']
        if time < start-1e-9 or time > end+1e-9 or (index and time <= start+1e-9):
            continue
        poses = {'A': np.array([point[k] for k in ['x_m', 'y_m', 'yaw_rad']])}
        for name, solution in solutions.items():
            s = solution(time)
            poses[name] = reconstruct(s) if name != 'C' else np.array([pose0[0]+s[0], pose0[1]+s[1], s[2]])
        errors = {}
        for name in ['A', 'B', 'D']:
            delta = poses[name]-poses['C']
            errors[name] = dict(position_m=float(np.linalg.norm(delta[:2])),
                                dx_m=float(delta[0]), dy_m=float(delta[1]),
                                yaw_rad=math.atan2(math.sin(delta[2]), math.cos(delta[2])))
        rows.append(dict(elapsed_sec=time, stage=index,
                         poses={name: pose.tolist() for name, pose in poses.items()}, errors_vs_cartesian=errors))
    start = end

# Compare derivatives of the first recorded segment to its frozen stage curvature.
_, _, h, dx, dy, actual_curvature = frame(origin)
report = {
    'scope': 'fixed controls, independent coordinate formulations; no solve, certificate or promotion',
    'source_sequence': e['source_sequence'], 'source_problem_fingerprint': e['source_problem_fingerprint'],
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'native_path_sha256': hashlib.sha256(native_path.read_bytes()).hexdigest(),
    'arms': {'A': 'native physical adapter', 'B': 'covariant arc-length equations with stored stage curvature',
             'C': 'Cartesian body ODE independent of virtual progress', 'D': 'covariant equations with actual course derivatives'},
    'first_segment': {'stored_stage_curvature': e['control_stages'][0]['path_curvature_radpm'],
                      'heading_derivative': actual_curvature,
                      'tangent_position_derivative': math.cos(h)*dx+math.sin(h)*dy,
                      'normal_position_derivative': -math.sin(h)*dx+math.cos(h)*dy},
    'summary': {}, 'rows': rows,
}
for horizon in [.055, .1, .2, start]:
    selected = [row for row in rows if row['elapsed_sec'] <= horizon+1e-9]
    report['summary'][str(horizon)] = {name: max(row['errors_vs_cartesian'][name]['position_m'] for row in selected) for name in ['A', 'B', 'D']}
out = run/'executed-replay/frame-comparison.json'
out.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k != 'rows'}, indent=2))
