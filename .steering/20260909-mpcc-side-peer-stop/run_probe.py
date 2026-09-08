"""Save separate Stop candidates and compare only their exact failed QPs."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering = Path('/task-steering/20260909-mpcc-side-peer-stop')
out = Path('/output/20260909-side-peer-stop-audit')
out.mkdir(exist_ok=False)
source = next(Path('/output/20260909-peer-envelope-dev2-r1/world-target-comparison-r2/396/candidate').glob('*/snapshot.yaml'))
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'), '-I/usr/include/eigen3',
           str(steering/'probe.cpp'), '-o', str(out/'probe')] + link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
results = []
for mode in ['zero', 'historical']:
    work = out/mode
    work.mkdir()
    with (work/'probe.log').open('w') as log:
        result = subprocess.run([str(out/'probe'),mode,str(source),str(work)], cwd=work,
                                stdout=log,stderr=subprocess.STDOUT,timeout=120)
    results.append(dict(mode=mode,return_code=result.returncode))
    print(json.dumps(results[-1]),flush=True)
    if result.returncode: raise RuntimeError(str(work/'probe.log'))
    for i,qp in enumerate(sorted((work/'mpcc_architecture_snapshots').glob('*/snapshot.yaml'))):
        target = work/f'qp-{i}'
        target.mkdir()
        with (target/'probe.log').open('w') as log:
            subprocess.run([str(out/'probe'),'qp',str(qp),str(target)], cwd=target,
                           stdout=log,stderr=subprocess.STDOUT,timeout=120,check=True)
        with (target/'physical-comparison.log').open('w') as log:
            subprocess.run([str(build/'mpcc_architecture_compare'),str(qp),'--kkt-equilibration-only'],
                           cwd=target,stdout=log,stderr=subprocess.STDOUT,timeout=120)
manifest = dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),results=results)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
