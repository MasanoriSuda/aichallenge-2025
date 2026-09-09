"""Unchanged bounded architectures on the new4264world; actual3695is captured."""
from pathlib import Path
import hashlib
import json
import subprocess,shlex

out=Path('/output/20260909-complete-rest-4264-domain-architectures')
out.mkdir(exist_ok=False)
source=next(Path('/output/20260909-complete-stop-dev2-r1/d1/mpcc_architecture_snapshots').glob('000000004264*/snapshot.yaml'))
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
copy=Path('/output/20260909-complete-rest-initial-domain/adapter_candidate.cpp')
program=out/'compare'
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(p/'src/mpcc_architecture_compare.cpp'),str(copy),'-o',str(program)]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True)

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
    baseline_commit='3c1cd03b',source=str(source),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    program_sha256=hashlib.sha256(program.read_bytes()).hexdigest(),
    limitation='fresh solves on current4264world; recorded3695execution is independently replayed without a solve',results=results),indent=2)+'\n')
