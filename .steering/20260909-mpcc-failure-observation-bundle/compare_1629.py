"""Unchanged bounded architectures on the new1629world; actual1100is captured."""
from pathlib import Path
import hashlib
import json
import subprocess

out=Path('/output/20260909-failure-bundle-1629-architectures')
out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-failure-bundle-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000001629*/snapshot.yaml'))
program=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare')
results=[]
for name,args in [('stop',['--stop-physical-support-only']),('normal',[])]:
    work=out/name;work.mkdir()
    with (work/'comparison.log').open('w') as log:
        try:
            completed=subprocess.run([str(program),str(source),*args],cwd=work,
                stdout=log,stderr=subprocess.STDOUT,timeout=180)
            result=dict(name=name,return_code=completed.returncode,args=args)
        except subprocess.TimeoutExpired:
            result=dict(name=name,args=args,result='inconclusive timeout after180s')
    results.append(result);print(json.dumps(result),flush=True)
(out/'manifest.json').write_text(json.dumps(dict(
    baseline_commit='6c4875ed',source=str(source),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    program_sha256=hashlib.sha256(program.read_bytes()).hexdigest(),
    limitation='fresh solves on current1629world; recorded1100execution is independently replayed without a solve',results=results),indent=2)+'\n')
