from pathlib import Path
import hashlib,json,shlex,subprocess
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-rear-peer-stop')
out=Path('/output/20260909-rear-peer-constraint-comparison');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I'+str(p/'src'),'-I/usr/include/eigen3',str(s/'compare_rear_constraint.cpp'),'-o',str(out/'compare')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
world=next(Path('/output/20260909-failure-bundle-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000001629*/snapshot.yaml'))
with (out/'native.log').open('w') as log:subprocess.run([str(out/'compare'),str(world),str(out/'report.yaml')],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
(out/'manifest.json').write_text(json.dumps(dict(world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),command=cmd,authority=False),indent=2)+'\n')
print((out/'native.log').read_text())
