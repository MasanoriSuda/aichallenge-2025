"""Frozen native nominal model, public initial states and prescribed commands.

Future private body/contact states are only targets. Future public commands are
exogenous test inputs, not information already known at the decision. Immediate
longitudinal publication and0.1ssteering delay here define the same diagnostic
input boundary as the preceding comparison, not guaranteed live application.
Post-Emergency/Finish windows test the plant under their actual changed command
schedule; they do not validate a counterfactual normal continuation.
Targets must remain inside the common source/command recording coverage. Physics
keeps running during teardown after ROS recording ends; that tail is not a known
public input schedule and is not eligible for prediction scoring.
"""
from pathlib import Path
import bisect
import hashlib
import importlib.util
import json
import math
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('corrected',
    Path('.steering/20260910-mpcc-shared-plant-model/compare_base_link_models.py'))
corrected = importlib.util.module_from_spec(spec); spec.loader.exec_module(corrected)
DT = .005
OUT = Path('output/20260910-empirical-plant-public-inputs-r3')
BINARY = Path('output/20260910-empirical-plant-native-r2/check')
PARAMETERS = Path('output/20260910-empirical-plant-native-r1/parameters.txt')


def evaluate(name, domain, identity, begin, end, moving_only):
    source = Path(f'output/20260909-force-step-ros-state-{name}-r1/{domain}.json')
    raw = json.loads(source.read_text()); by = {}
    for row in raw: by.setdefault(row['topic'].split('/')[-1], []).append(row)
    for series in by.values(): series.sort(key=lambda row: (row['stamp'], row['receipt']))
    requested_end = end
    coverage = {key: max(row['stamp'] for row in by[key])
                for key in ['imu_raw', 'velocity_status', 'steering_status', 'control_cmd', 'clock']}
    end = min(end, *coverage.values())
    def latest(key, stamp, receipt):
        return next((row for row in reversed(by[key])
                     if row['stamp'] <= stamp and row['receipt'] <= receipt), None)
    time, truth, tire = corrected.read_truth(name, identity)
    anchors = []; requests = []; skipped = {}
    for imu in by['imu_raw']:
        t = imu['stamp']; receipt = imu['receipt']
        if not begin <= t < end: continue
        velocity = latest('velocity_status', t, receipt)
        steering = latest('steering_status', t, receipt)
        if velocity is None or steering is None:
            skipped['missing_sensor'] = skipped.get('missing_sensor', 0)+1; continue
        if moving_only and velocity['v'] < .5: continue
        if max(t-velocity['stamp'], t-steering['stamp']) > .1:
            skipped['stale_sensor'] = skipped.get('stale_sensor', 0)+1; continue
        past = [row for row in by['control_cmd'] if row['stamp'] <= t and row['receipt'] <= receipt]
        if not past or past[0]['stamp'] > t-.11:
            skipped['incomplete_input_history'] = skipped.get('incomplete_input_history', 0)+1; continue
        future = [row for row in by['control_cmd'] if t < row['stamp'] <= t+2]
        commands = past+future
        stamps = [row['stamp']-t for row in commands]
        assert stamps == sorted(stamps)
        j0 = int(np.argmin(abs(time-t)))
        if abs(time[j0]-t) > 1e-7:
            skipped['no_exact_initial_target_epoch'] = skipped.get('no_exact_initial_target_epoch', 0)+1; continue
        steps = min(400, int((min(end, time[-1])-t-1e-7)/DT))
        if steps < 20: continue
        q = Rotation.from_quat([imu['orientation'][k] for k in 'xyzw'])*Rotation.from_euler('z', -math.pi/2)
        initial = [0., 0., q.as_euler('xyz')[2], velocity['v'], velocity['vy'],
                   imu['angular']['z'], 0., steering['steering']]
        identity_number = len(anchors)
        requests.append(f'{identity_number} {steps} '+' '.join(f'{v:.17g}' for v in initial))
        for k in range(steps):
            elapsed = k*DT
            longitudinal = bisect.bisect_right(stamps, elapsed)-1
            lateral = bisect.bisect_right(stamps, elapsed-.1)-1
            assert min(longitudinal, lateral) >= 0
            # Preserve the existing serializer gain: the imposed wire is
            # divided once into the kernel's semantic desired-steering input.
            requests.append(f'{DT:.17g} {commands[longitudinal]["acceleration"]:.17g} '
                            f'{commands[lateral]["wire_steering"]/1.435:.17g}')
        anchors.append(dict(t0=t, target_index=j0, imu_source_sec=t, imu_receipt_sec=receipt,
                            velocity_source_sec=velocity['stamp'], velocity_receipt_sec=velocity['receipt'],
                            steering_source_sec=steering['stamp'], steering_receipt_sec=steering['receipt'],
                            initial_forward_mps=velocity['v']))
    key = f'{name}-{domain}-{begin}-{end}'
    destination = OUT/key; destination.mkdir()
    content = PARAMETERS.read_text()+'\n'.join(requests)+'\n'
    (destination/'inputs.txt').write_text(content)
    (destination/'anchors.json').write_text(json.dumps(anchors, indent=2)+'\n')
    executed = subprocess.run([str(BINARY)], input=content, text=True, capture_output=True, check=True)
    (destination/'predictions.txt').write_text(executed.stdout)
    rows = []
    for line in executed.stdout.splitlines():
        values = [float(v) for v in line.split()]
        identifier, steps = (int(v) for v in values[:2]); predicted = np.array(values[2:])
        anchor = anchors[identifier]; target_time = anchor['t0']+steps*DT
        j = int(np.argmin(abs(time-target_time)))
        assert abs(time[j]-target_time) <= 1e-6 and target_time < end
        actual = truth[j].copy(); actual[:2] -= truth[anchor['target_index'], :2]
        error = predicted[:6]-actual
        error[2] = math.atan2(math.sin(error[2]), math.cos(error[2]))
        rows.append(dict(t0=anchor['t0'], steps=steps, initial_speed_mps=anchor['initial_forward_mps'],
                         actual_terminal_speed_mps=actual[3], predicted_terminal_speed_mps=predicted[3],
                         position_error_m=float(np.linalg.norm(error[:2])), yaw_error_rad=float(error[2]),
                         speed_error_mps=float(error[3]), lateral_error_mps=float(error[4]),
                         yaw_rate_error_radps=float(error[5]), tire_error_rad=float(predicted[7]-tire[j])))
    fields = ['position_error_m', 'yaw_error_rad', 'speed_error_mps', 'lateral_error_mps',
              'yaw_rate_error_radps', 'tire_error_rad']
    score = {str(h): {k: corrected.base.stats([row[k] for row in rows if row['steps'] == h])
                     for k in fields} for h in [20, 50, 100, 200, 400]}
    return dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                requested_window_sec=[begin, requested_end], window_sec=[begin, end],
                source_coverage_end_sec=coverage, moving_only=moving_only, anchors=len(anchors),
                skipped=skipped, score=score, rows=rows)


OUT.mkdir(exist_ok=False)
report = dict(authority=False, meaning=__doc__,
              binary_path=str(BINARY), source_files={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in [Path(__file__), Path('.steering/20260910-mpcc-empirical-plant/native_vehicle_model.cpp'),
                Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpcc_vehicle_model.cpp'),
                Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/mpcc_plant_awsim_2025.yaml')]}, binary_sha256=hashlib.sha256(BINARY.read_bytes()).hexdigest(),
              parameters_sha256=hashlib.sha256(PARAMETERS.read_bytes()).hexdigest(),
              single_holdout=evaluate('single', 'd1', -823441338, 35, 59.54, True),
              single_after_finish=evaluate('single', 'd1', -823441338, 59.54, 67.27, False),
              dev2={d: dict(pre_emergency=evaluate('dev2', d, i, 7.585, 9.709999782, True),
                            after_emergency_stop_restart=evaluate('dev2', d, i, 9.709999782, 24.3, False))
                    for d, i in [('d1', 1594692888), ('d2', -615963298)]})
previous = json.loads(Path('output/20260910-shared-plant-base-link-models-r1/report.json').read_text())
parity = []
for current, old in [(report['single_holdout'], previous['single_holdout']),
                     *[(report['dev2'][d]['pre_emergency'], previous['dev2'][d]) for d in ['d1', 'd2']]]:
    lookup = {(row['t0'], row['steps']): row for row in old['rows'] if row['model'] == 'dynamic_yaw'}
    for row in current['rows']:
        key = (row['t0'], row['steps'])
        if key not in lookup: continue
        for field in ['position_error_m', 'yaw_error_rad', 'speed_error_mps', 'tire_error_rad']:
            parity.append(abs(row[field]-lookup[key][field]))
report['native_previous_python_parity'] = corrected.base.stats(parity)
assert parity and max(parity) < 1e-10, report['native_previous_python_parity']
(OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
print('Native/Python moving parity', report['native_previous_python_parity'], flush=True)
for name, result in [('single', report['single_holdout']), ('post_finish', report['single_after_finish']),
                     *[(d+'-'+phase, value) for d, data in report['dev2'].items() for phase, value in data.items()]]:
    print(name, result['anchors'], result['skipped'],
          {h: {k: (v.get('mae'), v.get('maximum')) for k, v in score.items() if k != 'tire_error_rad'}
           for h, score in result['score'].items()}, flush=True)
