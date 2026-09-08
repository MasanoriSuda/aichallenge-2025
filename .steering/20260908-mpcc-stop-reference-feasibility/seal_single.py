"""Seal completed trial artifacts and print the independent acceptance fields."""
import hashlib
import json
from pathlib import Path
import sys

repetition = int(sys.argv[1])
assert repetition in (1, 2, 3)
root = Path(f'output/20260908-mpcc-stop-reference-single-r{repetition}')
paths = [root / name for name in ('manifest.json', 'mpcc-source.patch', 'monitor-summary.json',
                                  'log-summary.json', 'bag-analysis.json', 'd1/autoware.log',
                                  'd1/result-summary.json', 'd1/d1-result-details.json')]
paths += sorted((root / 'd1/rosbag2_autoware').glob('*'))
artifacts = []
for path in paths:
    if not path.is_file():
        continue
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1048576), b''):
            digest.update(chunk)
    artifacts.append(dict(path=str(path), sha256=digest.hexdigest(), size_bytes=path.stat().st_size))
steering = Path('.steering/20260908-mpcc-stop-reference-feasibility')
(steering / f'single-r{repetition}-evidence.json').write_text(
    json.dumps(dict(run_id=root.name, artifacts=artifacts), indent=2) + '\n')
logs = json.loads((root / 'log-summary.json').read_text())
details = logs['d1-result-details.json']
bag = json.loads((root / 'bag-analysis.json').read_text())['control_stats']
print(json.dumps(dict(run=root.name, finished=details['finished'], laps=details['lap_count'],
                      total_sec=details['total_lap_time'], penalty=details['penalty_count'],
                      callback=logs['callback'], control=bag,
                      moving_start_overrides=[d for d in logs['moving_override_decisions'] if d['vehicle_state'] == 'start'],
                      recovery_transitions=logs['recovery_transitions']), indent=2))
