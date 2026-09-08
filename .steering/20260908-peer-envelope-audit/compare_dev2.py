"""Sealed first-failure comparison; no source changes or physical authority."""
from pathlib import Path
import hashlib
import json
import subprocess
import yaml

run = Path('/output/20260909-peer-envelope-dev2-r1')
out = run/'first-failure-comparison'
out.mkdir(exist_ok=False)
program = '/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare'
results = []
cases = [(394,'published-source'),(396,'physical-proof-rejected'),(941,'normal-authority-unavailable')]
for sequence, kind in cases:
    source = next(p for p in (run/'d1/mpcc_architecture_snapshots').glob(f'{sequence:012d}-*/snapshot.yaml') if kind in str(p))
    data = yaml.safe_load(source.read_text())
    commands = [('architectures',[])]
    if sequence == 396:
        primal = data['production_outcome']['result']['primal']
        original = out/'396-original-primal.txt'
        original.write_text('\n'.join(format(v,'.17g') for v in primal)+'\n')
        semantic = list(primal)
        request = data['source']['semantic_request']
        semantic[:7] = request['initial_state'] + [request['current_steering_rad'], request['current_response_steering_rad']]
        changed = out/'396-semantic-initial-primal.txt'
        changed.write_text('\n'.join(format(v,'.17g') for v in semantic)+'\n')
        commands += [('original-physical',['--external-primal-physical-nonlinear-oracle',str(original)]),
                     ('semantic-initial-physical',['--external-primal-physical-nonlinear-oracle',str(changed)])]
    for name,args in commands:
        destination = out/f'{sequence}-{name}.log'
        with destination.open('w') as log:
            process = subprocess.run([program,str(source),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=90)
        results.append(dict(sequence=sequence,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                            comparison=name,return_code=process.returncode,log=str(destination)))
        print(json.dumps(results[-1]),flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
