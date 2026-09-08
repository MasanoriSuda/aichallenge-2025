"""Native fixed-source diagnostics and architecture comparison; no production change."""
from pathlib import Path
import json
import shlex
import subprocess
import yaml

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering = Path('/task-steering/20260908-peer-envelope-audit')
run = Path('/output/20260909-peer-envelope-dev2-r1')
out = run/'world-target-comparison-r2'
out.mkdir(exist_ok=False)
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'), '-I/usr/include/eigen3',
           str(steering/'prepare_world_candidates.cpp'), '-o', str(out/'prepare')] + link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
results = []
for sequence, kind in [(394,'published-source'),(396,'physical-proof-rejected'),(941,'normal-authority-unavailable')]:
    source = next(p for p in (run/'d1/mpcc_architecture_snapshots').glob(f'{sequence:012d}-*/snapshot.yaml') if kind in str(p))
    target = out/str(sequence)
    subprocess.run([str(out/'prepare'),str(source),str(target)],cwd=build,check=True,timeout=30)
    report = yaml.safe_load((target/'report.yaml').read_text())
    with (target/'architectures.log').open('w') as log:
        result = subprocess.run([str(build/'mpcc_architecture_compare'),report['candidate_snapshot']],
                                cwd=target,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    results.append(dict(sequence=sequence,return_code=result.returncode,report=report))
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results[-1]),flush=True)
