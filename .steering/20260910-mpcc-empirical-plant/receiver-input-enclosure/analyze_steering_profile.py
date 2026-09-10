"""Independent declared steering age check; private trace has no authority."""
from collections import Counter, defaultdict
from pathlib import Path
import hashlib
import json
import re
import shutil
import struct
import subprocess
import sys

run, out = map(Path, sys.argv[1:3])
out.mkdir(exist_ok=False)
design = Path(__file__).with_name('steering-observation-design.md')
manifest = json.loads((run / 'manifest.json').read_text())
assert design.stat().st_mtime < manifest['launch_started_wall_sec']
assert (run / 'artifact-restoration.json').exists()
longitudinal = subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_profile.py')),
                              str(run), str(out / 'longitudinal')], capture_output=True, text=True)
(out / 'longitudinal.log').write_text(longitudinal.stdout + longitudinal.stderr)

def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]

def signature(row):
    return row['id'], row['source_ns'], f32(row['wire_steering'])

events, errors = [], []
for line in (run / 'unity-player.log').read_text(errors='replace').splitlines():
    if not line.startswith('MPCC_INPUT_OBS '):
        continue
    if line.startswith('MPCC_INPUT_OBS error'):
        errors.append(line)
        continue
    row = dict(re.findall(r'(\w+)=([^ ]+)', line))
    row['event'] = line.split()[1]
    if row['event'] not in ('rx', 'steer'):
        continue
    for key in ('id', 'seq', 'source_ns', 'mono_ticks', 'ros_ns'):
        if key in row:
            row[key] = int(row[key])
    for key in ('wire_steering', 'steer_degrees'):
        if key in row:
            row[key] = float(row[key])
    events.append(row)
received = defaultdict(list)
for row in events:
    if row['event'] == 'rx':
        received[signature(row)].append(row)
receivers = []
for identity in sorted({r['id'] for r in events}):
    writes = [r for r in events if r['id'] == identity and r['event'] == 'steer']
    failures, ambiguities, intervals = [], [], []
    counts = Counter(signature(row) for row in writes)
    for key, count in counts.items():
        if count > len(received[key]):
            failures.append(dict(reason='more writes than matching receives', signature=key, count=count))
    for row in writes:
        matches = [r for r in received[signature(row)] if r['mono_ticks'] <= row['mono_ticks']]
        # Decoded Unity TryGetAckermannSteerInput: negate then multiply float32
        # radians by its exact ldc.r4 degrees conversion constant.
        expected = f32(-f32(row['wire_steering']) * f32(57.295780181884766))
        if not matches or expected != f32(row['steer_degrees']):
            failures.append(dict(reason='steering receive/payload mismatch', write=row, expected_degrees=expected))
        if len(matches) > 1:
            ambiguities.append(dict(write=row, possible_receive_sequences=[r['seq'] for r in matches]))
        if row['source_ns'] > row['ros_ns']:
            failures.append(dict(reason='source later than assignment clock', write=row))
    for before, after in zip(writes, writes[1:]):
        age = after['ros_ns'] - before['source_ns']
        item = dict(start_ros_ns=before['ros_ns'], end_ros_ns=after['ros_ns'], source_ns=before['source_ns'],
                    maximum_packet_age_ns=age, wire_steering=before['wire_steering'])
        intervals.append(item)
        ordered = (before['source_ns'] <= after['source_ns'] and
                   before['mono_ticks'] < after['mono_ticks'] and
                   before['source_ns'] <= before['ros_ns'] <= after['ros_ns'])
        if not ordered:
            failures.append(dict(reason='source/assignment order invalid', interval=item))
        if not 0 <= age <= 100_000_000:
            failures.append(dict(reason='outside predeclared 100ms steering profile', interval=item))
    receivers.append(dict(receiver_id=identity, assignments=len(writes), complete_intervals=len(intervals),
                          maximum_age_interval=max(intervals, key=lambda r:r['maximum_packet_age_ns']) if intervals else None,
                          failures=failures, ambiguous_receive_identities=ambiguities, intervals=intervals,
                          passed=bool(intervals) and not failures,
                          outside_250ms_observation=sum(r['maximum_packet_age_ns'] > 250_000_000 for r in intervals)))
report = dict(authority=False, run=run.name, steering_receipt_profile_ms=100,
              steering_mechanical_delay_ms=100, longitudinal_profile_ms=250,
              longitudinal_passed=longitudinal.returncode == 0,
              steering_passed=not errors and len(receivers) == 2 and all(r['passed'] for r in receivers),
              predeclaration_mtime_sec=design.stat().st_mtime, launch_started_wall_sec=manifest['launch_started_wall_sec'],
              probe_errors=errors, receivers=receivers,
              limitations=['Private generated instrumentation, not runtime acceptance or a transport guarantee.',
                           'Initial pre-assignment and final open intervals are censored.',
                           'Duplicate receive identities are retained; source and float payload establish the age coordinate.',
                           'No bound on body/model error, physical contact or arbitrary thread scheduling.'])
(out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
(out / 'events.json').write_text(json.dumps(events, indent=2) + '\n')
shutil.copy2(design, out / 'predeclared-design.md')
(out / 'manifest.json').write_text(json.dumps(dict(files={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (Path(__file__), design, run / 'unity-player.log', run / 'manifest.json', run / 'artifact-restoration.json')}), indent=2) + '\n')
print(json.dumps({k:v for k,v in report.items() if k != 'receivers'}, indent=2))
for row in receivers:
    print(json.dumps({k:v for k,v in row.items() if k not in ('intervals','ambiguous_receive_identities')}, indent=2))
raise SystemExit(0 if report['longitudinal_passed'] and report['steering_passed'] else 1)
