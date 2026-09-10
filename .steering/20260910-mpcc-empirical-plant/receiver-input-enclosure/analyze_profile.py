"""Validate the predeclared 250 ms empirical profile on complete logged intervals."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import struct
import sys

run, out = map(Path, sys.argv[1:3])
out.mkdir(exist_ok=False)
design = Path(__file__).with_name('design.md')
manifest = json.loads((run / 'manifest.json').read_text())
assert (run / 'artifact-restoration.json').exists()
predeclared = design.stat().st_mtime < manifest['launch_started_wall_sec']
assert run.name != '20260910-nine-state-application-dev2-r3' or predeclared
events, errors = [], []
for line in (run / 'unity-player.log').read_text(errors='replace').splitlines():
    if not line.startswith('MPCC_INPUT_OBS '):
        continue
    if line.startswith('MPCC_INPUT_OBS error'):
        errors.append(line)
        continue
    row = dict(re.findall(r'(\w+)=([^ ]+)', line))
    row['event'] = line.split()[1]
    for key in ('id', 'seq', 'source_ns', 'mono_ticks', 'ros_ns'):
        if key in row:
            row[key] = int(row[key])
    for key in ('input', 'selected_input', 'unity', 'selected_unity'):
        if key in row:
            row[key] = float(row[key])
    events.append(row)
received = {(r['id'], r['seq']): r for r in events if r['event'] == 'rx'}
assert len(received) == sum(r['event'] == 'rx' for r in events)
receivers = []
for identity in sorted({r['id'] for r in events}):
    selected = [r for r in events if r['id'] == identity and r['event'] == 'apply']
    failures, intervals = [], []
    for row in selected:
        rx = received.get((identity, row['seq']))
        same = (rx is not None and rx['source_ns'] == row['source_ns'] and
                rx['mono_ticks'] <= row['mono_ticks'] and row['emergency'] == 'False' and
                row['unity'] == row['selected_unity'] and
                struct.pack('f', rx['input']) == struct.pack('f', row['input']) ==
                struct.pack('f', row['selected_input']))
        if not same:
            failures.append(dict(reason='selection mismatch', selection=row, receipt=rx))
    for before, after in zip(selected, selected[1:]):
        age = after['ros_ns'] - before['source_ns']
        interval = dict(start_ros_ns=before['ros_ns'], end_ros_ns=after['ros_ns'],
                        source_ns=before['source_ns'], sequence=before['seq'],
                        maximum_packet_age_ns=age, input=before['input'])
        intervals.append(interval)
        ordered = (before['seq'] < after['seq'] and
                   before['source_ns'] <= after['source_ns'] and
                   before['mono_ticks'] < after['mono_ticks'] and
                   before['source_ns'] <= before['ros_ns'] < after['ros_ns'])
        if not ordered:
            failures.append(dict(reason='time/sequence order invalid', interval=interval))
        if not 0 <= age <= 250_000_000:
            failures.append(dict(reason='outside predeclared 250ms profile', interval=interval))
    receivers.append(dict(receiver_id=identity, selected=len(selected),
                          complete_intervals=len(intervals),
                          maximum_age_interval=max(intervals, key=lambda x: x['maximum_packet_age_ns'])
                          if intervals else None, failures=failures,
                          passed=bool(intervals) and not failures, intervals=intervals))
report = dict(authority=False, run=run.name, empirical_profile_ms=250,
              independent_predeclaration=predeclared,
              predeclaration_mtime_sec=design.stat().st_mtime,
              launch_started_wall_sec=manifest['launch_started_wall_sec'],
              passed_complete_observed_intervals=(not errors and len(receivers) == 2 and
                                                 all(r['passed'] for r in receivers)),
              probe_errors=errors, receivers=receivers,
              limitations=['Private instrumented receiver trace is diagnostic only.',
                           'The initial pre-selection and final open intervals are unobserved/censored.',
                           'ROS source time is the tested coordinate, not a wall-time transport guarantee.',
                           'Body/model error and steering receipt are not bounded by this experiment.',
                           'No real-vehicle guarantee, runtime model adoption, or race acceptance.'])
(out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
(out / 'events.json').write_text(json.dumps(events, indent=2) + '\n')
shutil.copy2(design, out / 'predeclared-design.md')
(out / 'manifest.json').write_text(json.dumps(dict(files={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (Path(__file__), design, run / 'unity-player.log', run / 'manifest.json',
              run / 'artifact-restoration.json')}), indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if k != 'receivers'}, indent=2))
for row in receivers:
    print(json.dumps({k: v for k, v in row.items() if k != 'intervals'}, indent=2))
raise SystemExit(0 if report['passed_complete_observed_intervals'] else 1)
