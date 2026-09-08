"""Compare A/B/C/D and fresh Stop on newly captured current-world failure 993."""
from pathlib import Path
import hashlib
import json
import subprocess

build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-published-stop-architectures')
out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-published-stop-observation-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000000993*/snapshot.yaml'))
program=build/'mpcc_architecture_compare'
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
