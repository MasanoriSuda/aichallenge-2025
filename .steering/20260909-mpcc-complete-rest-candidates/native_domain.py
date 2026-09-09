from pathlib import Path
import shlex,subprocess,sys,json
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-complete-rest-domain-native-'+sys.argv[1]);out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_adapter.dir/link.txt').read_text())
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',str(p/'test/test_mpcc_rate_resolved_adapter.cpp'),'-o',str(out/'regression')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True)
with (out/'test.log').open('w') as log:r=subprocess.run([str(out/'regression'),'--gtest_filter=*InitialTangentRespectsCourseDomainWithoutChangingObjectiveOrBounds'],cwd=b,stdout=log,stderr=subprocess.STDOUT)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode),indent=2)+'\n')
print((out/'test.log').read_text());sys.exit(r.returncode)
