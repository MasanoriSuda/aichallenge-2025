from pathlib import Path
import hashlib,json,shlex,subprocess,shutil
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-peer-viability-native-recorder-r2');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_snapshot.dir/link.txt').read_text())
sources=[p/'test/test_mpcc_architecture_snapshot.cpp',p/'src/mpcc_architecture_snapshot.cpp']
for path in [*sources,p/'include/multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp']:
    shutil.copy2(path,out/path.name)
cmd=[link[0],'-std=c++17','-O2','-pthread','-I'+str(p/'include'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',*map(str,sources),'-o',str(out/'regression')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'regression')],cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=60)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True);r.check_returncode()
