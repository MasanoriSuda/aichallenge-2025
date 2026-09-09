from pathlib import Path
import subprocess,shlex,hashlib,json
from domain_candidate import transform
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
s=Path('/task-steering/20260909-mpcc-complete-rest-candidates')
out=Path('/output/20260909-complete-rest-initial-domain');out.mkdir(exist_ok=False)
src=p/'src/mpcc_rate_resolved_adapter.cpp'
copy=out/'adapter_candidate.cpp';copy.write_text(transform(src.read_text()))
link=shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_adapter.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',str(p/'test/test_mpcc_rate_resolved_adapter.cpp'),str(copy),'-o',str(out/'regression')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True)
with (out/'test.log').open('w') as log:r=subprocess.run([str(out/'regression')],cwd=b,stdout=log,stderr=subprocess.STDOUT)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),copy_sha256=hashlib.sha256(copy.read_bytes()).hexdigest(),authority=False),indent=2)+'\n')
print((out/'test.log').read_text());r.check_returncode()
