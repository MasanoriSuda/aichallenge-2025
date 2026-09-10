"""Same-run packet selection and fixed-body-model input-schedule comparison.

Prescribed future inputs score the plant only. No replay here can authorize a
command or repair a different run's missing simulator application observation.
"""
from pathlib import Path
import bisect
import collections
import hashlib
import json
import re
import struct
import subprocess
import sys
import numpy as np

run, inputs, out = map(Path, sys.argv[1:4])
assert (run/'artifact-restoration.json').exists()
out.mkdir(exist_ok=False)
events = []
for line in (run/'unity-player.log').read_text(errors='replace').splitlines():
    if not line.startswith('MPCC_INPUT_OBS '):
        continue
    assert not line.startswith('MPCC_INPUT_OBS error'), line
    row = dict(re.findall(r'(\w+)=([^ ]+)', line))
    row['event'] = line.split()[1]
    for key in ('id', 'seq', 'source_ns', 'mono_ticks', 'ros_ns'):
        if key in row:
            row[key] = int(row[key])
    for key in ('input', 'unity', 'selected_input', 'selected_unity'):
        if key in row:
            row[key] = float(row[key])
    events.append(row)
rx = [row for row in events if row['event'] == 'rx']
applied = [row for row in events if row['event'] == 'apply']
lookup = {(row['id'], row['seq']): row for row in rx}
assert len(lookup) == len(rx)
bits = lambda x: struct.pack('f', x)
for row in applied:
    original = lookup[row['id'], row['seq']]
    assert row['emergency'] == 'False'
    assert original['source_ns'] == row['source_ns']
    assert bits(original['input']) == bits(row['selected_input']) == bits(row['input'])
    assert original['mono_ticks'] <= row['mono_ticks']
    assert row['unity'] == row['selected_unity']
public = {d: json.loads((inputs/(d+'.json')).read_text()) for d in ('d1', 'd2')}
keys = {d: {(r['stamp_ns'], bits(r['acceleration'])) for r in rows if 'wire_steering' in r}
        for d, rows in public.items()}
stats = lambda values: dict(count=len(values), minimum=float(np.min(values)),
    median=float(np.median(values)), maximum=float(np.max(values)),
    mae=float(np.mean(np.abs(values)))) if values else dict(count=0)
receivers = []
for identity in sorted({row['id'] for row in rx}):
    received = [row for row in rx if row['id'] == identity]
    selected = [row for row in applied if row['id'] == identity]
    unique = collections.Counter()
    for row in received:
        matches = [d for d, wire in keys.items() if (row['source_ns'], bits(row['input'])) in wire]
        if len(matches) == 1:
            unique[matches[0]] += 1
    # Identical source stamp/float acceleration can occur on both cars; one
    # recorder missing that packet makes a collision appear falsely unique.
    # Infer the receiver-level mapping from the full signature and retain all
    # contradictory matches. This is diagnostic association, not per-packet
    # proof of Domain identity or command authority.
    ranked = unique.most_common()
    assert ranked and ranked[0][1] > 100, unique
    assert ranked[0][1] > sum(count for _, count in ranked[1:]), unique
    domain = ranked[0][0]
    used = {row['seq'] for row in selected}
    transitions = []
    for before, after in zip(selected, selected[1:]):
        if before['input'] <= 0 or after['input'] >= 0:
            continue
        first = next(row for row in received if before['seq'] < row['seq'] <= after['seq'] and row['input'] < 0)
        transitions.append(dict(first_received=first, selected=after,
            first_brake_source_to_apply_sec=(after['ros_ns']-first['source_ns'])*1e-9,
            overwritten_sequences=[row['seq'] for row in received if first['seq'] <= row['seq'] < after['seq']]))
    receivers.append(dict(domain=domain, receiver_id=identity, unique_matches=dict(unique),
        domain_association='dominant full-run stamp/float signature; minority collisions retained in unique_matches',
        received=len(received), applied=len(selected),
        overwritten_before_last_apply=sum(row['seq'] not in used for row in received if row['seq'] < max(used)),
        selected_source_to_apply_sec=stats([(row['ros_ns']-row['source_ns'])*1e-9 for row in selected]),
        apply_intervals_sec=stats([(b['ros_ns']-a['ros_ns'])*1e-9 for a, b in zip(selected, selected[1:])]),
        braking_transitions=transitions))

assert len({row['domain'] for row in receivers}) == len(receivers) == len(public), 'Receiver mapping must be one-to-one'

binary = Path('output/20260910-empirical-plant-native-r2/check')
parameters = Path('output/20260910-empirical-plant-native-r1/parameters.txt')
requests, anchors = [], []
for receiver in receivers:
    domain = receiver['domain']
    by = collections.defaultdict(list)
    for row in public[domain]:
        by[row['topic'].split('/')[-1]].append(row)
    for rows in by.values():
        rows.sort(key=lambda row: (row['stamp'], row['receipt']))
    selected = [row for row in applied if row['id'] == receiver['receiver_id']]
    selected_stamps = [row['ros_ns']*1e-9 for row in selected]
    commands = by['control_cmd']
    command_stamps = [row['stamp'] for row in commands]
    velocities = by['velocity_status']
    for i, velocity in enumerate(velocities):
        t = velocity['stamp']
        if velocity['v'] < .1 or t < command_stamps[0]+.2:
            continue
        recent = lambda kind: next((r for r in reversed(by[kind]) if r['stamp'] <= t and r['receipt'] <= velocity['receipt']), None)
        imu, tire = recent('imu_raw'), recent('steering_status')
        if imu is None or tire is None or max(t-imu['stamp'], t-tire['stamp']) > .1:
            continue
        for distance in (1, 2, 4, 8):
            if i+distance >= len(velocities):
                continue
            target = velocities[i+distance]
            duration = target['stamp']-t
            if duration > .5 or target['stamp'] > min(command_stamps[-1], selected_stamps[-1]):
                continue
            count = int(np.ceil(duration/.005))
            dt = duration/count
            crosses_brake = any(t <= r['selected']['ros_ns']*1e-9 <= target['stamp'] for r in receiver['braking_transitions'])
            for schedule in ('publication-source', 'observed-application'):
                identity = len(anchors)
                initial = [0, 0, 0, velocity['v'], velocity['vy'], imu['angular']['z'], 0, tire['steering']]
                requests.append(f'{identity} {count} '+' '.join(f'{v:.17g}' for v in initial))
                for k in range(count):
                    stamp = t+k*dt
                    lateral = bisect.bisect_right(command_stamps, stamp-.1)-1
                    assert lateral >= 0
                    if schedule == 'publication-source':
                        wire = commands[bisect.bisect_right(command_stamps, stamp)-1]['acceleration']
                    else:
                        j = bisect.bisect_right(selected_stamps, stamp)-1
                        assert j >= 0
                        wire = selected[j]['input']
                    requests.append(f'{dt:.17g} {wire:.17g} {commands[lateral]["wire_steering"]/1.435:.17g}')
                anchors.append(dict(domain=domain, schedule=schedule, source_sec=t,
                    target_sec=target['stamp'], count=count, target_u=target['v'],
                    crosses_observed_brake=crosses_brake))
content = parameters.read_text()+'\n'.join(requests)+'\n'
(out/'inputs.txt').write_text(content)
executed = subprocess.run([str(binary)], input=content, text=True, capture_output=True, check=True)
(out/'predictions.txt').write_text(executed.stdout)
scored = []
for line in executed.stdout.splitlines():
    values = [float(v) for v in line.split()]
    anchor = anchors[int(values[0])]
    if int(values[1]) != anchor['count']:
        continue
    scored.append(dict(**anchor, predicted_u=values[5], error_u=values[5]-anchor['target_u']))
scores = {d: {phase: {schedule: stats([r['error_u'] for r in scored if r['domain'] == d and
    r['schedule'] == schedule and (phase == 'all' or r['crosses_observed_brake'])])
    for schedule in ('publication-source', 'observed-application')}
    for phase in ('all', 'crossing-brake')} for d in public}
report = dict(authority=False, run=str(run), receivers=receivers, scores=scores,
    limitations=['Instrumented run changes timing and is not race acceptance.',
        'Direct application is a posthoc prescribed input, not available to the participant.',
        'Source skew and unknown steering receipt/application remain in both arms.',
        'This run does not reconstruct dev2-r12 application events or certify its counterfactual commands.'])
(out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
(out/'scored.json').write_text(json.dumps(scored, indent=2)+'\n')
(out/'receiver-events.json').write_text(json.dumps(events, indent=2)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(authority=False, files={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (Path(__file__), run/'unity-player.log', inputs/'d1.json', inputs/'d2.json', binary, parameters)}), indent=2)+'\n')
print(json.dumps(dict(receivers=receivers, scores=scores), indent=2))
