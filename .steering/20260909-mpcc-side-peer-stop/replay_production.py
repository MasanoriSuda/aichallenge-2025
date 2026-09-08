"""Revisit the sealed world only after its numerical and coordinate producer changed."""
from pathlib import Path
import hashlib
import json
import subprocess

build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-side-peer-production-replay')
out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-peer-envelope-dev2-r1/world-target-comparison-r2/396/candidate').glob('*/snapshot.yaml'))
program=build/'mpcc_architecture_compare'
with (out/'build.log').open('w') as log:
    subprocess.run(['cmake','--build','.','--target','mpcc_architecture_compare','-j','4'],cwd=build,
                   stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
results=[]
for name,args in [('stop',['--stop-physical-support-only']),('normal',[])]:
    work=out/name
    work.mkdir()
    with (work/'comparison.log').open('w') as log:
        completed=subprocess.run([str(program),str(source),*args],cwd=work,
                                 stdout=log,stderr=subprocess.STDOUT,timeout=180)
    results.append(dict(name=name,return_code=completed.returncode,args=args))
    print(json.dumps(results[-1]),flush=True)
(out/'manifest.json').write_text(json.dumps(dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    program_sha256=hashlib.sha256(program.read_bytes()).hexdigest(),results=results),indent=2)+'\n')
