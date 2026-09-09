from pathlib import Path
import hashlib,json,shlex,subprocess
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-stop-proof-provenance')
source=Path('/output/20260909-complete-rest-dev2-r1/d2-executed-328/snapshot.yaml')
out=Path('/output/20260909-stop-provenance-328-original-r2');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I'+str(p/'src'),'-I/usr/include/eigen3',str(s/'replay_original_328.cpp'),'-o',str(out/'replay'),'libmulti_purpose_mpc_ros_mpcc_rate_resolved_certified_plan.a']+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'replay'),str(source),str(out/'report.yaml')],cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=60)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),solver_invocations=0,authority=False,limits='Original observed world of actual 328 only; current 923 steering state is not captured. This is not a current-state join or a reconstruction of missing Stop 395.'),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True);r.check_returncode()
