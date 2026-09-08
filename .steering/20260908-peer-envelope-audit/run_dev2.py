"""One sealed bounded dev2 trial of the complete physical peer envelope."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import time

steering = Path(__file__).resolve().parent
repo = Path.cwd()
package = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
root = Path('output/20260909-peer-envelope-dev2-r1')
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Other containers running'
assert '100% tests passed, 0 tests failed out of 60' in Path('/tmp/mpcc-20260909-peer-package.log').read_text()
root.mkdir(parents=True, exist_ok=False)
template = json.loads(Path('.steering/20260909-mpcc-coordinate-consistency/run-manifest-template.json').read_text())
manifest = dict(run_id=root.name, kind='initial-dev2-six-lap-full-peer-body-feasibility',
                baseline_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                initial_conditions='standard dev2 with6laps/600s only; noNPCs/collisions on/handicap off/wall recovery off',
                seed='AWSIM default; explicit seed unavailable, no determinism claimed',
                termination='both finished, first active moving Emergency/Recovery, or840s host deadline',
                command=f'make dev2 LOG_DIR=/{root}', files=[], binaries=[])
paths = {item['path'] for item in template['files']}
paths.update(subprocess.check_output(['git','diff','--name-only','--',str(package)],text=True).splitlines())
paths.add(str(package/'test/fixtures/awsim_peer_body_envelope.json'))
paths.add('aichallenge/run_autoware.bash')
paths.add('aichallenge/run_simulator.bash')
paths.add('Makefile')
for path in sorted(paths):
    src = Path(path)
    dst = root/'inputs'/src
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    manifest['files'].append(dict(path=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest(),preserved=str(dst)))
(root/'binaries').mkdir()
for item in template['binaries']:
    src = Path(item['local_real_path'])
    dst = root/'binaries'/src.name
    shutil.copy2(src,dst)
    manifest['binaries'].append(dict(path=str(src),sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),preserved=str(dst)))
for kind in ['build','package']:
    shutil.copy2('/tmp/mpcc-20260909-peer-'+kind+'.log',root/(kind+'.log'))
patch = subprocess.check_output(['git','diff','--',str(package)])
(root/'mpcc-source.patch').write_bytes(patch)
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
for src in [Path(__file__),script,overlay]:
    manifest['files'].append(dict(path=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(steering/'dev2-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
summary = {}
termination = '840s host deadline'
try:
    with (root/'launch.log').open('w') as log:
        subprocess.run(['make','dev2',f'LOG_DIR=/{root}'],env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    containers = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    manifest['running_images'] = {name: subprocess.check_output(['docker','inspect','--format','{{.Image}}',name],text=True).strip() for name in containers}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (steering/'dev2-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    deadline = time.monotonic()+840
    while time.monotonic() < deadline:
        failed,complete = False,True
        for domain in ['d1','d2']:
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
            try:
                result = json.loads((root/domain/(domain+'-result-details.json')).read_text())
            except (OSError,json.JSONDecodeError):
                result = {}
            complete &= result.get('finished',False)
            if result: summary[domain]['result'] = result
        print(json.dumps(summary),flush=True)
        if failed:
            termination = 'first active moving Emergency/Recovery; preserve earliest failure'
            time.sleep(2)
            break
        if complete:
            termination = 'both finished; inspect penalties and coverage'
            time.sleep(10)
            break
        time.sleep(10)
except KeyboardInterrupt:
    termination = 'external interruption; consult external-stop-reason.json'
    raise
finally:
    (root/'monitor-summary.json').write_text(json.dumps(dict(termination=termination,last_observation=summary),indent=2)+'\n')
    with (root/'down.log').open('w') as log:
        for project in ['1','2',None]:
            command = ['docker','compose']+(['-p',project] if project else [])+['down','--remove-orphans']
            subprocess.run(command,env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
    print(termination,flush=True)
