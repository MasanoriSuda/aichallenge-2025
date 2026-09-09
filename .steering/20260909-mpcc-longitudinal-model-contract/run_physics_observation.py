"""Bounded diagnostic run with receiver/application probes; no race acceptance claim."""
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
assert attempt in ['r1','r2','r3']
assert mode == 'dev2'
target = 'dev' if mode=='single' else 'dev2'
domains = ['d1'] if mode=='single' else ['d1','d2']
projects = ['1'] if mode=='single' else ['1','2']
root = Path('output/20260909-longitudinal-physics-observed-'+mode+'-'+attempt)
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Other containers running'
assert '100% tests passed, 0 tests failed out of 60' in Path('/tmp/mpcc-stop-viability-package.log').read_text()
assert 'Summary: 2381 tests, 0 errors, 0 failures, 0 skipped' in Path('/tmp/mpcc-stop-viability-package.log').read_text()
root.mkdir(parents=True, exist_ok=False)
template = json.loads(Path('.steering/20260909-mpcc-coordinate-consistency/run-manifest-template.json').read_text())
manifest = dict(run_id=root.name, kind='physical-parameter-observation-'+mode,
                baseline_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                initial_conditions='standard '+mode+' with6laps/600s only; noNPCs/collisions on/handicap off/wall recovery off',
                seed='AWSIM default; explicit seed unavailable, no determinism claimed',
                termination='all vehicles finished, first active moving Emergency/Recovery, or120s host diagnostic deadline',
                command=f'make {target} LOG_DIR=/{root}', files=[], binaries=[])
paths = {item['path'] for item in template['files']}
paths.update(subprocess.check_output(['git','diff','--name-only','--',str(package)],text=True).splitlines())
paths.update(subprocess.check_output(['git','diff','--name-only','--','aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser'],text=True).splitlines())
paths.add('aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser/test/test_gnss_heading.cpp')
paths.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard','--',str(package)],text=True).splitlines())
paths.add(str(package/'test/fixtures/awsim_peer_body_envelope.json'))
paths.add(str(package/'test/fixtures/mpcc_zero_stop_qp.yaml'))
paths.add('aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so')
for extra_package in ['aichallenge_ekf_localizer','aichallenge_submit_launch']:
    extra = Path('aichallenge/workspace/src/aichallenge_submit')/extra_package
    paths.update(str(x) for x in extra.rglob('*') if x.is_file())
paths.add('aichallenge/workspace/build/aichallenge_ekf_localizer/libaichallenge_ekf_localizer_lib.so')
paths.add('aichallenge/workspace/build/aichallenge_ekf_localizer/ekf_localizer')
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
template['binaries'].append(dict(local_real_path=str(package).replace('/src/aichallenge_submit/','/build/')+'/libmulti_purpose_mpc_ros_mpc_state_prediction.so'))
for item in template['binaries']:
    src = Path(item['local_real_path'])
    dst = root/'binaries'/src.name
    shutil.copy2(src,dst)
    manifest['binaries'].append(dict(path=str(src),sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),preserved=str(dst)))
for kind, log in [('build','/tmp/mpcc-stop-viability-build.log'), ('package','/tmp/mpcc-stop-viability-package.log')]:
    shutil.copy2(log,root/(kind+'.log'))
patch = subprocess.check_output(['git','diff','--',str(package),'aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch'])
(root/'mpcc-source.patch').write_bytes(patch)
manifest['control_baseline'] = '1c4f377e'
manifest['scope'] = 'Minimum read-only runtime physics parameters at moving application: mass/inertia/COM/rolling/skid/wheel sprung masses and positions. Participant remains1c4f377e. Same instrumented main DLL, extra one-time helper reflection. No model/config/authority/solver change; not race acceptance.'
manifest['working_patch_sha256'] = hashlib.sha256(patch).hexdigest()
original = Path('aichallenge/simulator_scripts/dev.sh').read_text()
assert '--laps unlimited' in original and '--timeout 10000000.0' in original
script = root/'dev-six-lap.sh'
script.write_text(original.replace('--laps unlimited','--laps 6').replace('--timeout 10000000.0','--timeout 600').replace('--lidar off', '--lidar off -logFile /'+str(root)+'/unity-player.log'))
overlay = root/'compose-six-lap.yml'
overlay.write_text('services:\n  simulator:\n    volumes:\n      - '+str((repo/script).resolve())+':/aichallenge/simulator_scripts/dev.sh:ro\n')
probe_root = Path('output/20260909-longitudinal-physics-instrumentation-r1')
validation = json.loads((probe_root/'cil-validation-r2.json').read_text())
assert not validation['differences'] and len(validation['probe_calls']) == 4
assert hashlib.sha256((probe_root/'Assembly-CSharp.dll').read_bytes()).hexdigest() == validation['instrumented_sha256']
original_dll = Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll')
assert hashlib.sha256(original_dll.read_bytes()).hexdigest() == validation['original_sha256']
manifest['instrumentation'] = dict(validation=validation, files=[])
for probe_name in ['Assembly-CSharp.dll','ObservationProbe.dll']:
    probe_path = probe_root/probe_name
    target_path = '/aichallenge/simulator/AWSIM/AWSIM_Data/Managed/'+probe_name
    with overlay.open('a') as overlay_file:
        overlay_file.write('      - '+str((repo/probe_path).resolve())+':'+target_path+':ro\n')
    shutil.copy2(probe_path, root/'binaries'/('probe-'+probe_name))
    manifest['instrumentation']['files'].append(dict(path=str(probe_path),sha256=hashlib.sha256(probe_path.read_bytes()).hexdigest()))
shutil.copy2(original_dll,root/'binaries/Assembly-CSharp-original.dll')
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
(steering/(mode+'-'+attempt+'-manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')
summary = {}
termination = '120s host diagnostic deadline'
try:
    manifest['launch_started_wall_sec'] = time.time()
    with (root/'launch.log').open('w') as log:
        subprocess.run(['make',target,f'LOG_DIR=/{root}'],env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    containers = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    manifest['runtime_mounts'] = {name: json.loads(subprocess.check_output(['docker','inspect','--format','{{json .Mounts}}',name],text=True)) for name in containers}
    manifest['running_images'] = {name: subprocess.check_output(['docker','inspect','--format','{{.Image}}',name],text=True).strip() for name in containers}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (steering/(mode+'-'+attempt+'-manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')
    deadline = time.monotonic()+120
    while time.monotonic() < deadline:
        failed,complete = False,True
        simulator_log = (root/'unity-player.log').read_text(errors='replace') if (root/'unity-player.log').exists() else ''
        physical = [line for line in simulator_log.splitlines() if line.startswith('MPCC_PHYSICS_OBS ')]
        physical_ids = set(re.search(r'id=(-?\d+)',line).group(1) for line in physical)
        if len(physical_ids)==len(domains) and all(
            len([line for line in physical if ('id='+identity+' ') in line and ' wheel=' in line])==4
            for identity in physical_ids):
            termination = 'minimum physical parameters captured for all receivers; diagnostic only'
            summary['physics_records'] = physical
            break
        if ('MPCC_INPUT_OBS error' in simulator_log or
            'FileNotFoundException' in simulator_log or 'InvalidProgramException' in simulator_log):
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
            failed |= first_override is not None or any(' -> Recovery,' in line for line in phases)
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
            termination = 'first active moving Emergency/Recovery; preserve earliest failure'
            time.sleep(2)
            break
        if complete:
            termination = 'all vehicles finished; inspect penalties and coverage'
            time.sleep(1)
            break
        time.sleep(1)
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
    assert hashlib.sha256(original_dll.read_bytes()).hexdigest() == validation['original_sha256']
    print(termination,flush=True)
