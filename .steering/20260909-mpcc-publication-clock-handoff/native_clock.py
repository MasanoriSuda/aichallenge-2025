from pathlib import Path
import shlex,subprocess,sys,json,hashlib
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-publication-clock-native-'+sys.argv[1]);out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_certified_plan.dir/link.txt').read_text())
sources=[p/'test/test_mpcc_rate_resolved_certified_plan.cpp',p/'src/mpcc_rate_resolved_certified_plan.cpp']
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',*map(str,sources),'-o',str(out/'regression')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'test.log').open('w') as log:r=subprocess.run([str(out/'regression'),'--gtest_filter=*'],cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}),indent=2)+'\n')
print((out/'test.log').read_text(),flush=True);sys.exit(r.returncode)
