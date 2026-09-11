"""Frozen empirical MPCC runs; application probes are explicitly diagnostic only."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
import sys

steering = Path(__file__).resolve().parent
repo = Path.cwd()
package = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
mode = sys.argv[1]
attempt = sys.argv[2] if len(sys.argv)>2 else 'r1'
timing_diagnostic = len(sys.argv)>3 and sys.argv[3]=='timing'
application_diagnostic = len(sys.argv)>3 and sys.argv[3]=='application'
assert len(sys.argv)<=3 or timing_diagnostic or application_diagnostic
assert re.fullmatch(r'r[1-9][0-9]*', attempt)
assert mode in ['single','dev2','dev3','dev4']
target = 'dev' if mode=='single' else mode
vehicle_count = 1 if mode == 'single' else int(mode[-1])
domains = [f'd{i}' for i in range(1, vehicle_count + 1)]
projects = [str(i) for i in range(1, vehicle_count + 1)]
run_kind = 'application-' if application_diagnostic else ''
root = Path('output/20260910-nine-state-'+run_kind+mode+'-'+attempt)
host_duration_sec = 120 if application_diagnostic else 840
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Other containers running'
test_log = Path('/tmp/mpcc-nine-state-tests-r68.log')
build_log = Path('/tmp/mpcc-nine-state-build-r79.log')
assert 'Summary: 2510 tests, 0 errors, 0 failures, 0 skipped' in test_log.read_text()
assert 'Summary: 26 packages finished' in build_log.read_text()
original_dll = Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll')
assert hashlib.sha256(original_dll.read_bytes()).hexdigest() == '703e18fad4e3cf68111a559190edb7060e901988a04c409d84c80331dd45a172'
root.mkdir(parents=True, exist_ok=False)
template = json.loads(Path('.steering/20260909-mpcc-coordinate-consistency/run-manifest-template.json').read_text())
manifest = dict(run_id=root.name, kind='empirical-nine-state-'+mode,
                baseline_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                initial_conditions='standard '+mode+' with6laps/600s only; noNPCs/collisions on/handicap off/wall recovery off',
                seed='AWSIM default; explicit seed unavailable, no determinism claimed',
                termination=f'all vehicles finished, first active moving Emergency/Recovery, or{host_duration_sec}s host deadline',
                command=f'make {target} LOG_DIR=/{root}', timing_diagnostic=timing_diagnostic,
                application_diagnostic=application_diagnostic,
                tactical_rejoin_policy='Record committed-pass longitudinal watchdog Recovery phase separately; final moving Emergency/Recovery overrides and every other Recovery entry still terminate', files=[], binaries=[])
paths = {item['path'] for item in template['files']}
paths.update(str(x) for x in package.rglob('*') if x.is_file() and '__pycache__' not in x.parts)
paths.update(subprocess.check_output(['git','diff','--name-only','--','aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser'],text=True).splitlines())
paths.add('aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser/test/test_gnss_heading.cpp')
paths.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard','--',str(package)],text=True).splitlines())
paths.add(str(package/'test/fixtures/awsim_peer_body_envelope.json'))
paths.add(str(package/'test/fixtures/mpcc_zero_stop_qp.yaml'))
paths.add('aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so')
for extra_package in ['aichallenge_ekf_localizer','aichallenge_submit_launch','racing_kart_gnss_poser','imu_gnss_poser','racing_kart_sensor_kit_description']:
    extra = Path('aichallenge/workspace/src/aichallenge_submit')/extra_package
    paths.update(str(x) for x in extra.rglob('*') if x.is_file())
paths.add('aichallenge/workspace/build/aichallenge_ekf_localizer/libaichallenge_ekf_localizer_lib.so')
paths.add('aichallenge/workspace/build/aichallenge_ekf_localizer/ekf_localizer')
paths.add('aichallenge/workspace/build/imu_gnss_poser/libimu_gnss_poser_heading_reference.so')
paths.add('aichallenge/workspace/build/imu_gnss_poser/imu_gnss_poser_node')
paths.add('aichallenge/run_autoware.bash')
paths.add('aichallenge/run_simulator.bash')
paths.add('Makefile')
for path in sorted(paths):
    src = Path(path)
    dst = root/'inputs'/src
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    manifest['files'].append(dict(path=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest(),preserved=str(dst)))
manifest['protected_user_artifacts'] = []
for src in [Path('aichallenge/result-summary.json'), Path('aichallenge/safety-gate-result.json')]:
    dst = root/'user-inputs'/src.name
    dst.parent.mkdir(exist_ok=True)
    shutil.copy2(src,dst)
    manifest['protected_user_artifacts'].append(dict(path=str(src), backup=str(dst), sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
(root/'binaries').mkdir()
template['binaries'].append(dict(local_real_path=str(package).replace('/src/aichallenge_submit/','/build/')+'/libmulti_purpose_mpc_ros_mpcc_vehicle_model.so'))
for item in template['binaries']:
    src = Path(item['local_real_path'])
    dst = root/'binaries'/src.name
    shutil.copy2(src,dst)
    manifest['binaries'].append(dict(path=str(src),sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),preserved=str(dst)))
for kind, log in [('build', build_log), ('package', test_log)]:
    shutil.copy2(log, root/(kind+'.log'))
shutil.copy2(original_dll, root/'binaries/Assembly-CSharp-original.dll')
patch = subprocess.check_output(['git','diff','--','aichallenge/workspace/src/aichallenge_submit'])
(root/'mpcc-source.patch').write_bytes(patch)
manifest['control_baseline'] = manifest['baseline_commit']
manifest['scope'] = 'Required original-source applied-input certificate with empirical250ms receiver profile, same common float program/full-rest Stop and final packet guard. Shared nine-state COM/tire/body model, causal public velocity/IMU/tire and serialized input history, model-bound QP/nonlinear/Stop/artifact/async, terminal rest proof. Fixed 2025 empirical parameters. No physical margin/tolerance/solver budget relaxation. All integrated acceptance remains subject to actual run results.'
manifest['working_patch_sha256'] = hashlib.sha256(patch).hexdigest()
original = Path('aichallenge/simulator_scripts/dev.sh').read_text()
assert '--laps unlimited' in original and '--timeout 10000000.0' in original
script = root/'dev-six-lap.sh'
script.write_text(original.replace('--laps unlimited','--laps 6').replace('--timeout 10000000.0','--timeout 600'))
if application_diagnostic:
    script.write_text(script.read_text().replace('--lidar off', '--lidar off -logFile /'+str(root)+'/unity-player.log'))
overlay = root/'compose-six-lap.yml'
overlay.write_text('services:\n  simulator:\n    volumes:\n      - '+str((repo/script).resolve())+':/aichallenge/simulator_scripts/dev.sh:ro\n')
if application_diagnostic:
    probe_root = Path('output/20260911-steering-instrumentation-r2')
    validation = json.loads((probe_root/'cil-validation-r3.json').read_text())
    assert not validation['differences'] and len(validation['probe_calls']) == 5
    assert hashlib.sha256(original_dll.read_bytes()).hexdigest() == validation['original_sha256']
    assert hashlib.sha256((probe_root/'Assembly-CSharp.dll').read_bytes()).hexdigest() == validation['instrumented_sha256']
    manifest['instrumentation'] = dict(validation=validation, files=[])
    manifest['acceptance'] = 'diagnostic only; added receiver/application logging changes timing'
    for name in ('Assembly-CSharp.dll', 'ObservationProbe.dll'):
        source = probe_root/name
        with overlay.open('a') as stream:
            stream.write('      - '+str((repo/source).resolve())+':/aichallenge/simulator/AWSIM/AWSIM_Data/Managed/'+name+':ro\n')
        shutil.copy2(source, root/'binaries'/('probe-'+name))
        manifest['instrumentation']['files'].append(dict(path=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
environment = os.environ.copy()
base = environment.get('COMPOSE_FILE')
if not base:
    base = next(line.split('=',1)[1].strip().strip('"\'') for line in Path('.env').read_text().splitlines() if line.startswith('COMPOSE_FILE='))
environment['COMPOSE_FILE'] = base+':'+str(overlay)
manifest['compose_file'] = environment['COMPOSE_FILE']
shutil.copy2(__file__,root/'run_campaign.py')
for src in [Path(__file__),script,overlay]:
    manifest['files'].append(dict(path=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(steering/(run_kind+mode+'-'+attempt+'-manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')
summary = {}
termination = f'{host_duration_sec}s host deadline'
try:
    manifest['launch_started_wall_sec'] = time.time()
    with (root/'launch.log').open('w') as log:
        subprocess.run(['make',target,f'LOG_DIR=/{root}'],env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    containers = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    manifest['runtime_mounts'] = {name: json.loads(subprocess.check_output(['docker','inspect','--format','{{json .Mounts}}',name],text=True)) for name in containers}
    manifest['running_images'] = {name: subprocess.check_output(['docker','inspect','--format','{{.Image}}',name],text=True).strip() for name in containers}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (steering/(run_kind+mode+'-'+attempt+'-manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')
    deadline = time.monotonic()+host_duration_sec
    while time.monotonic() < deadline:
        failed,complete = False,True
        if application_diagnostic:
            player = root/'unity-player.log'
            player_text = player.read_text(errors='replace') if player.exists() else ''
            if any(error in player_text for error in ('MPCC_INPUT_OBS error', 'FileNotFoundException', 'InvalidProgramException')):
                termination = 'instrumentation error; inconclusive diagnostic'
                break
        for domain in domains:
            log = root/domain/'autoware.log'
            lines = re.sub(r'\x1b\[[0-9;]*m','',log.read_text(errors='replace')).splitlines() if log.exists() else []
            phases = [re.sub(r'\x1b\[[0-9;]*m','',line) for line in lines if 'OvertakeLine:' in line and ' -> ' in line]
            first_override = None
            vehicle_state = None
            for line in lines:
                observed = re.search(r'AWSIM vehicle state observed: state=(\S+)', line)
                if observed:
                    vehicle_state = observed.group(1)
                if vehicle_state not in ('ready', 'start'):
                    continue
                if 'Overtake control decision:' in line:
                    authority = re.search(r'MPCC execution contract:.*?authority=([^,]+)', line)
                    speed = re.search(r'actual=([^m]+)m/s', line)
                    if (authority and speed and
                        authority.group(1) in ('emergency-override', 'recovery-override') and
                        abs(float(speed.group(1))) > .1):
                        first_override = re.sub(r'\x1b\[[0-9;]*m','',line)
                        break
                if ('Stuck recovery: mode=active' in line and 'action=' in line and
                    'action=NormalControl' not in line):
                    first_override = re.sub(r'\x1b\[[0-9;]*m','',line)
                    break
            stops = [line for line in lines if 'canonical-certified-terminal-stop' in line]
            laps = [line for line in lines if 'Lap ' in line and 'completed!' in line]
            summary[domain] = dict(lines=len(lines),phases=phases[-3:],laps=laps[-1:],certified_stop_traces=len(stops),
                                   stop_executed_traces=sum('executed-retained' in line for line in stops),
                                   first_active_override=first_override)
            launch_errors = [line for line in lines if '[ERROR] [launch]' in line]
            summary[domain]['body_prediction_samples'] = sum('MPCC body prediction:' in line for line in lines)
            summary[domain]['causal_input_failsafes'] = sum('missing causal body/tire/input observation' in line for line in lines)
            publication_violations = [line for line in lines if 'MPCC publication window violated:' in line]
            summary[domain]['publication_window_violations'] = publication_violations
            summary[domain]['certified_publications'] = sum('MPCC applied publication:' in line for line in lines)
            if launch_errors:
                summary[domain]['launch_errors'] = launch_errors
            tactical_rejoins = [line for line in phases if 'Pass -> Recovery,' in line and
                                'reason=committed pass longitudinal progress stalled,' in line]
            unexpected_recovery = [line for line in phases if ' -> Recovery,' in line and line not in tactical_rejoins]
            summary[domain]['tactical_rejoin_transitions'] = tactical_rejoins
            summary[domain]['unexpected_recovery_transitions'] = unexpected_recovery
            failed |= bool(launch_errors) or first_override is not None or bool(unexpected_recovery) or bool(publication_violations)
            if timing_diagnostic and vehicle_state in ('ready', 'start'):
                raw = [(int(i), float(ms)) for line in lines if 'Control callback samples:' in line
                       for i, ms in re.findall(r'(\d+):([\d.]+)', line.split('decision_elapsed_ms=', 1)[1])]
                pairs = [(a, b) for a, b in zip(raw, raw[1:]) if b[0] == a[0]+1 and min(a[1], b[1]) > 25.0]
                shift_decisions = [int(re.search(r'decision=(\d+)', line)[1]) for line in lines
                                   if 'Overtake control decision:' in line and 'phase=ShiftOut,' in line]
                pairs = [pair for pair in pairs if shift_decisions and pair[0][0] >= min(shift_decisions)]
                summary[domain]['adjacent_callback_overruns'] = pairs
            result = {}
            # AWSIM writes here at Finish; orchestrator collects into root only
            # during teardown. Require this run's mtime and observed Finish so
            # an older vehicle JSON cannot prematurely complete the campaign.
            source_result = Path('aichallenge')/(domain+'-result-details.json')
            if vehicle_state == 'finish':
                try:
                    if source_result.stat().st_mtime >= manifest['launch_started_wall_sec']:
                        candidate = json.loads(source_result.read_text())
                        if candidate.get('finished') and candidate.get('lap_count') == 6:
                            destination = root/'finish-results'/source_result.name
                            destination.parent.mkdir(exist_ok=True)
                            shutil.copy2(source_result, destination)
                            result = candidate
                except (OSError,json.JSONDecodeError):
                    pass
            complete &= result.get('finished',False)
            if result: summary[domain]['result'] = result
        print(json.dumps(summary),flush=True)
        if failed:
            termination = ('launch failure; controller acceptance not evaluated' if
                any('launch_errors' in row for row in summary.values()) else
                'first authority/publication-window failure; preserve earliest failure')
            time.sleep(2)
            break
        if timing_diagnostic and any(row.get('adjacent_callback_overruns') for row in summary.values()):
            termination = 'first observed adjacent callback overruns; timing attribution only, no race acceptance'
            time.sleep(2)
            break
        if complete:
            termination = 'all vehicles finished; inspect penalties and coverage'
            time.sleep(10)
            break
        time.sleep(10)
except KeyboardInterrupt:
    termination = 'external interruption; consult external-stop-reason.json'
    raise
finally:
    (root/'monitor-summary.json').write_text(json.dumps(dict(termination=termination,last_observation=summary,shutdown_started_wall_sec=time.time()),indent=2)+'\n')
    try:
        with (root/'down.log').open('w') as log:
            jobs = [subprocess.Popen(['docker','compose','-p',project,'down','--remove-orphans'],
                     env=environment,stdout=log,stderr=subprocess.STDOUT) for project in projects]
            codes = [job.wait() for job in jobs]
            subprocess.run(['docker','compose','down','--remove-orphans'],env=environment,
                           stdout=log,stderr=subprocess.STDOUT,check=True)
            assert all(code == 0 for code in codes), codes
    finally:
        receipt = []
        for item in manifest['protected_user_artifacts']:
            src = Path(item['path'])
            current = hashlib.sha256(src.read_bytes()).hexdigest() if src.exists() else None
            if current != item['sha256']:
                generated = root/'generated-user-path-results'/src.name
                generated.parent.mkdir(exist_ok=True)
                if src.exists(): shutil.copy2(src,generated)
                shutil.copy2(item['backup'],src)
            restored = hashlib.sha256(src.read_bytes()).hexdigest()
            assert restored == item['sha256']
            receipt.append(dict(path=str(src), generated_sha256=current, restored_sha256=restored))
        (root/'artifact-restoration.json').write_text(json.dumps(receipt,indent=2)+'\n')
    assert hashlib.sha256(original_dll.read_bytes()).hexdigest() == '703e18fad4e3cf68111a559190edb7060e901988a04c409d84c80331dd45a172'
    print(termination,flush=True)
