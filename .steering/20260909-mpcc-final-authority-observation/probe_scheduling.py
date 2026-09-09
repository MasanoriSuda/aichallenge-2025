from pathlib import Path
import hashlib,json,shlex,subprocess
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-final-authority-observation')
out=Path('/output/20260909-final-authority-scheduling');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_snapshot.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-pthread','-I'+str(p/'include'),'-I'+str(p/'test'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',str(s/'probe_scheduling.cpp'),str(p/'src/latest_only_worker.cpp'),'-o',str(out/'probe')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'probe'),str(out/'report.yaml')],cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=15)
(out/'manifest.json').write_text(json.dumps(dict(baseline_commit='e5b8c455',command=cmd,return_code=r.returncode,authority=False,sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [p/'src/latest_only_worker.cpp',p/'src/mpcc_architecture_snapshot.cpp',p/'src/mpc_controller_cpp.cpp']}),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True);r.check_returncode()
