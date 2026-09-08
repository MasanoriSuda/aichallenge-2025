"""Offline only: rebuild the saved artifact's physical path, with no solver call."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import sys
import yaml

run = Path(sys.argv[1])
package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering = Path('/task-steering/20260908-mpcc-delay-prefix-audit')
out = run/'executed-replay'
out.mkdir(exist_ok=False)
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
# Reuse the exact installed build's dependency list; replace only the CLI main.
object_index = next(i for i, value in enumerate(link) if value.endswith('mpcc_architecture_compare.cpp.o'))
output_index = link.index('-o')
assert output_index == object_index + 1
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'),
           '-I/usr/include/eigen3', str(steering/'reconstruct_execution.cpp'),
           '-o', str(out/'reconstruct_execution')] + link[output_index+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
records = []
for path in (run/'d1/mpcc_architecture_snapshots').glob('*/snapshot.yaml'):
    data = yaml.safe_load(path.read_text())
    if 'execution_evidence' not in data:
        continue
    evidence = data['execution_evidence']
    assert evidence['source_sequence'] == data['source']['sequence']
    assert evidence['source_problem_fingerprint'] == data['source']['problem_context']['fingerprint']
    source_id = str(data['source']['sequence'])
    destination = out/(source_id+'-exact-physical.yaml')
    with (out/(source_id+'-native.log')).open('w') as log:
        process = subprocess.run([str(out/'reconstruct_execution'), str(path), str(destination)],
                                 cwd=build, stdout=log, stderr=subprocess.STDOUT, timeout=60)
    item = dict(source_sequence=int(source_id), input=str(path),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                native_return_code=process.returncode, output=str(destination),
                publication=evidence['publication'],
                trajectory_role=('exact-executed-source' if evidence['publication']['source_kind']=='exact-executed' else 'bundle-source-only-not-the-published-bundle'))
    # A full solved horizon can also enter the existing exact physical oracle.
    # A shorter executed prefix cannot be invented into a full-horizon primal.
    stages = evidence['control_stages']
    if len(stages) == data['source']['semantic_request']['horizon_steps']:
        values = []
        for state in evidence['predicted_states']:
            values.extend(state[k] for k in ['lateral_m','lag_m','heading_offset_rad','velocity_mps','progress_m','steering_rad','response_steering_rad'])
        for control in stages:
            values.extend(control[k] for k in ['acceleration_mps2','steering_rate_radps','virtual_progress_speed_mps'])
        primal = out/(source_id+'-actual-primal.txt')
        primal.write_text('\n'.join(format(value,'.17g') for value in values)+'\n')
        with (out/(source_id+'-physical-oracle.log')).open('w') as log:
            oracle = subprocess.run([str(build/'mpcc_architecture_compare'),str(path),
                '--external-primal-physical-nonlinear-oracle',str(primal)],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=60)
        item['physical_oracle_return_code'] = oracle.returncode
    records.append(item)
(out/'results.json').write_text(json.dumps(records,indent=2)+'\n')
print(json.dumps(records,indent=2))
