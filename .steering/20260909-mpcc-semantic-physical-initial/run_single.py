"""One bounded six-lap acceptance after separate physical initial-state repair."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import time

repo = Path.cwd()
steering = Path(__file__).resolve().parent
root = Path('output/20260909-semantic-initial-single-r1')
name = 'semantic-initial-20260909-r1'
assert not subprocess.check_output(['docker','ps','-q']).strip(), 'Other containers running'
assert '100% tests passed, 0 tests failed out of 60' in Path('/tmp/mpcc-20260909-semantic-initial-separated-package.log').read_text()
root.mkdir(exist_ok=False)
template=json.loads(Path('.steering/20260909-mpcc-coordinate-consistency/run-manifest-template.json').read_text())
package='aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'
paths={item['path'] for item in template['files']}
paths.update(subprocess.check_output(['git','diff','--name-only','--',package],text=True).splitlines())
paths.add(package+'/test/fixtures/awsim_peer_body_envelope.json')
paths.update(['aichallenge/run_autoware.bash','aichallenge/run_simulator.bash','Makefile',
              'aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so'])
manifest=dict(run_id=root.name,baseline_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              scope='single-vehicle physical seed/authority/timing acceptance; coupled/submission incomplete',
              repetitions=1,termination='six-lap finish, first active moving override, or420s host deadline',
              files=[],binaries=[],protected_user_artifacts=[])
for text in sorted(paths):
    src=Path(text)
    dst=root/'inputs'/src
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    manifest['files'].append(dict(path=str(src),sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),preserved=str(dst)))
(root/'binaries').mkdir()
for item in template['binaries']:
    src=Path(item['local_real_path'])
    dst=root/'binaries'/src.name
    shutil.copy2(src,dst)
    manifest['binaries'].append(dict(path=str(src),sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),preserved=str(dst)))
for src in [Path('aichallenge/result-summary.json'),Path('aichallenge/safety-gate-result.json')]:
    dst=root/'user-inputs'/src.name
    dst.parent.mkdir(exist_ok=True)
    shutil.copy2(src,dst)
    manifest['protected_user_artifacts'].append(dict(path=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest(),backup=str(dst)))
for kind in ['build','package']:
    shutil.copy2('/tmp/mpcc-20260909-semantic-initial-separated-'+kind+'.log',root/(kind+'.log'))
patch=subprocess.check_output(['git','diff','--',package])
(root/'mpcc-source.patch').write_bytes(patch)
manifest['working_patch_sha256']=hashlib.sha256(patch).hexdigest()
logdir='/'+str(root)+'/d1'
command=("source /aichallenge/workspace/install/setup.bash && "
         f"export ROS_HOME={logdir}/ros && export ROS_LOG_DIR={logdir}/ros/log && "
         f"mkdir -p {logdir}/ros/log && cd {logdir} && "
         "exec ros2 launch aichallenge_system_launch evaluation.launch.xml "
         "domain_id:=1 vehicle_count:=1 sim_mode:=eval "
         f"log_dir:={logdir} capture:=true rosbag:=true simulation:=true "
         f"use_sim_time:=true run_rviz:=true > {logdir}/autoware.log 2>&1")
manifest['command']=command
manifest['runner_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
shutil.copy2(__file__,root/'run_single.py')
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(steering/'single-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
environment=os.environ.copy()
environment.update(LOG_DIR='/'+str(root),ROS_DOMAIN_ID='1',AIC_VEHICLE_COUNT='1',CMD=command)
subprocess.run(['docker','compose','run','-d','--rm','--name',name,'--no-deps','autoware-command'],env=environment,check=True)
manifest['image_id']=subprocess.check_output(['docker','inspect','--format','{{.Image}}',name],text=True).strip()
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(steering/'single-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
deadline=time.monotonic()+420
termination='420s host deadline'
summary={}
try:
    while time.monotonic()<deadline:
        log=root/'d1/autoware.log'
        lines=re.sub(r'\x1b\[[0-9;]*m','',log.read_text(errors='replace')).splitlines() if log.exists() else []
        state=None
        failure=None
        for line in lines:
            observed=re.search(r'AWSIM vehicle state observed: state=(\S+)',line)
            if observed: state=observed.group(1)
            if state not in ('ready','start'): continue
            if 'Overtake control decision:' in line:
                authority=re.search(r'MPCC execution contract:.*?authority=([^,]+)',line)
                speed=re.search(r'actual=([^m]+)m/s',line)
                if authority and speed and authority.group(1) in ('emergency-override','recovery-override') and abs(float(speed.group(1)))>.1:
                    failure=line
                    break
            if 'Stuck recovery: mode=active' in line and 'action=NormalControl' not in line:
                failure=line
                break
        laps=[line for line in lines if 'Lap ' in line and 'completed!' in line]
        summary=dict(lines=len(lines),state=state,laps=laps[-1:],first_active_override=failure)
        print(json.dumps(summary),flush=True)
        if failure:
            termination='first active moving Emergency/Recovery'
            time.sleep(2)
            break
        try: result=json.loads((root/'d1/result-summary.json').read_text())
        except (OSError,json.JSONDecodeError): result={}
        if result.get('vehicles') and all(v.get('finished') for v in result['vehicles']):
            termination='six laps finished'
            summary['result']=result
            time.sleep(10)
            break
        time.sleep(10)
except KeyboardInterrupt:
    termination='external interruption'
    raise
finally:
    (root/'monitor-summary.json').write_text(json.dumps(dict(termination=termination,last_observation=summary),indent=2)+'\n')
    subprocess.run(['docker','stop','-t','30',name],check=True)
    print(termination,flush=True)
