"""Unchanged bounded architecture comparisons on the immutable 923 world."""
from pathlib import Path
import hashlib,json,subprocess

out=Path('/output/20260909-stop-provenance-923-comparison');out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-complete-rest-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000000923*/snapshot.yaml'))
program=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare')
results=[]
for name,args in [('stop',['--stop-physical-support-only']),('normal',[])]:
    work=out/name;work.mkdir()
    with (work/'comparison.log').open('w') as log:
        try:
            completed=subprocess.run([str(program),str(source),*args],cwd=work,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            result=dict(name=name,return_code=completed.returncode,args=args)
        except subprocess.TimeoutExpired:result=dict(name=name,args=args,result='inconclusive timeout after 180 seconds')
    results.append(result);print(json.dumps(result),flush=True)
(out/'manifest.json').write_text(json.dumps(dict(baseline_commit='ae6862aa',source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),program_sha256=hashlib.sha256(program.read_bytes()).hexdigest(),limitation='Fresh independent candidates, never substitutes for recorded 328 or missing 395. All-method failure is Unknown without a physical infeasibility proof.',results=results),indent=2)+'\n')
