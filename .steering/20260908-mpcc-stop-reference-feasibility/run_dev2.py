"""Fixed-binary, scoped dev2 diagnostic/acceptance continuation."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

steering = Path('.steering/20260908-mpcc-stop-reference-feasibility')
for repetition in (1, 2, 3):
    single = Path(f'output/20260908-mpcc-stop-reference-single-r{repetition}')
    details = json.loads((single / 'd1/d1-result-details.json').read_text())
    assert details['finished'] and details['lap_count'] == 6 and details['penalty_count'] == 0
    log_summary = json.loads((single / 'log-summary.json').read_text())
    assert not log_summary['recovery_transitions']
    assert not any(d['vehicle_state'] == 'start' for d in log_summary['moving_override_decisions'])
    assert (single / 'bag-analysis.json').exists()
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Other containers running'
root = Path('output/20260908-mpcc-stop-reference-dev2-r1')
root.mkdir(parents=True, exist_ok=False)
manifest = json.loads(Path('.steering/20260908-mpcc-stateless-successor/dev2-manifest.json').read_text())
protocol = json.loads((steering / 'protocol.json').read_text())
for item in manifest['files']:
    assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['sha256'], item['path']
(root / 'binaries').mkdir()
for item in manifest['binaries']:
    source = Path(item['local_real_path'])
    destination = root / 'binaries' / source.name
    shutil.copy2(source, destination)
    item['sha256'] = hashlib.sha256(destination.read_bytes()).hexdigest()
    assert item['sha256'] == protocol['binaries'][str(source)]
    item['preserved_path'] = str(destination)
package = 'aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'
patch = subprocess.check_output(['git', 'diff', '--', package])
(root / 'mpcc-source.patch').write_bytes(patch)
env = os.environ.copy()
base_compose = env.get('COMPOSE_FILE')
if not base_compose:
    base_compose = next(l.split('=', 1)[1].strip().strip('"\'') for l in Path('.env').read_text().splitlines()
                        if l.startswith('COMPOSE_FILE='))
env['COMPOSE_FILE'] = base_compose + ':' + str(steering / 'compose-six-lap.yml')
for path in (steering / 'compose-six-lap.yml', steering / 'dev-six-lap.sh', Path(__file__)):
    manifest['files'].append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
manifest.update(run_id=root.name, kind='dev2-six-lap-scoped-workspace-acceptance',
                command=f'make dev2 LOG_DIR=/{root}', compose_file=env['COMPOSE_FILE'],
                working_tree_patch_sha256=hashlib.sha256(patch).hexdigest(),
                build_log='/tmp/mpcc-20260908-stop-reference-build.log',
                tests='output/20260908-mpcc-stop-reference-feasibility/package-test-results.log',
                termination='All vehicles finished, tactical Recovery, or840s host deadline',
                initial_conditions='Standard dev2 with only laps=6 and timeout=600 via steering-only simulator script mount; original files unchanged')
(root / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
(steering / 'dev2-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
termination = '840s host deadline'
summary = {}
try:
    with (root / 'launch.log').open('w') as log:
        subprocess.run(['make', 'dev2', f'LOG_DIR=/{root}'], env=env, check=True, stdout=log, stderr=subprocess.STDOUT)
    deadline = time.monotonic() + 840
    while time.monotonic() < deadline:
        failed, complete = False, True
        for domain in ('d1', 'd2'):
            log = root / domain / 'autoware.log'
            lines = log.read_text(errors='replace').splitlines() if log.exists() else []
            phases = [re.sub(r'\x1b\[[0-9;]*m', '', s) for s in lines if 'OvertakeLine:' in s and ' -> ' in s]
            stop = [s for s in lines if 'canonical-certified-terminal-stop' in s]
            summary[domain] = dict(lines=len(lines), phases=phases[-4:],
                                   certified_stop_traces=len(stop),
                                   stop_executed_traces=sum('executed-retained' in s for s in stop))
            failed |= any(' -> Recovery,' in s for s in phases)
            result_path = root / domain / f'{domain}-result-details.json'
            try:
                result = json.loads(result_path.read_text())
            except (OSError, json.JSONDecodeError):
                result = {}
            complete &= result.get('finished', False)
            if result:
                summary[domain]['result'] = result
        print(json.dumps(summary), flush=True)
        if failed:
            termination = 'tactical Recovery; seal earliest failure before repeating'
            time.sleep(2)
            break
        if complete:
            termination = 'both vehicles finished; inspect penalties and acceptance coverage'
            time.sleep(10)
            break
        time.sleep(20)
finally:
    (root / 'monitor-summary.json').write_text(json.dumps(dict(termination=termination, last_observation=summary), indent=2) + '\n')
    with (root / 'down.log').open('w') as log:
        for project in ('1', '2', None):
            command = ['docker', 'compose'] + (['-p', project] if project else []) + ['down', '--remove-orphans']
            subprocess.run(command, env=env, check=True, stdout=log, stderr=subprocess.STDOUT)
    print(termination, flush=True)
