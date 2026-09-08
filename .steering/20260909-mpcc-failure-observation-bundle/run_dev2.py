"""Bounded dev2 observation-only capture after atomic failure bundle repair."""
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
assert mode == 'dev2', 'Observation-only diagnostic follows the already tested single baseline'
target = 'dev' if mode=='single' else 'dev2'
domains = ['d1'] if mode=='single' else ['d1','d2']
projects = ['1'] if mode=='single' else ['1','2']
root = Path('output/20260909-failure-bundle-'+mode+'-'+attempt)
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Other containers running'
assert '100% tests passed, 0 tests failed out of 60' in Path('/tmp/mpcc-failure-bundle-package.log').read_text()
assert 'Summary: 2362 tests, 0 errors, 0 failures, 0 skipped' in Path('/tmp/mpcc-failure-bundle-package.log').read_text()
root.mkdir(parents=True, exist_ok=False)
template = json.loads(Path('.steering/20260909-mpcc-coordinate-consistency/run-manifest-template.json').read_text())
manifest = dict(run_id=root.name, kind='failure-bundle-observation-'+mode+'-acceptance',
                baseline_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                initial_conditions='standard '+mode+' with6laps/600s only; noNPCs/collisions on/handicap off/wall recovery off',
                seed='AWSIM default; explicit seed unavailable, no determinism claimed',
                termination='all vehicles finished, first active moving Emergency/Recovery, or840s host deadline',
                command=f'make {target} LOG_DIR=/{root}', files=[], binaries=[])
paths = {item['path'] for item in template['files']}
paths.update(subprocess.check_output(['git','diff','--name-only','--',str(package)],text=True).splitlines())
paths.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard','--',str(package)],text=True).splitlines())
paths.add(str(package/'test/fixtures/awsim_peer_body_envelope.json'))
paths.add(str(package/'test/fixtures/mpcc_zero_stop_qp.yaml'))
paths.add('aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so')
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
for kind, log in [('build','/tmp/mpcc-failure-bundle-full-build.log'), ('package','/tmp/mpcc-failure-bundle-package.log')]:
    shutil.copy2(log,root/(kind+'.log'))
patch = subprocess.check_output(['git','diff','--',str(package)])
(root/'mpcc-source.patch').write_bytes(patch)
manifest['unchanged_control_baseline'] = '6c4875ed4d250a44cd813935b234863e9c184701'
manifest['scope'] = 'observation writer and source-bundle changes only; no controller model/solver/parameter changes'
manifest['working_patch_sha256'] = hashlib.sha256(patch).hexdigest()
original = Path('aichallenge/simulator_scripts/dev.sh').read_text()
assert '--laps unlimited' in original and '--timeout 10000000.0' in original
script = root/'dev-six-lap.sh'
script.write_text(original.replace('--laps unlimited','--laps 6').replace('--timeout 10000000.0','--timeout 600'))
overlay = root/'compose-six-lap.yml'
overlay.write_text('services:\n  simulator:\n    volumes:\n      - '+str((repo/script).resolve())+':/aichallenge/simulator_scripts/dev.sh:ro\n')
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
termination = '840s host deadline'
try:
    manifest['launch_started_wall_sec'] = time.time()
    with (root/'launch.log').open('w') as log:
        subprocess.run(['make',target,f'LOG_DIR=/{root}'],env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    containers = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    manifest['running_images'] = {name: subprocess.check_output(['docker','inspect','--format','{{.Image}}',name],text=True).strip() for name in containers}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (steering/(mode+'-'+attempt+'-manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')
    deadline = time.monotonic()+840
    while time.monotonic() < deadline:
        failed,complete = False,True
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
    print(termination,flush=True)
