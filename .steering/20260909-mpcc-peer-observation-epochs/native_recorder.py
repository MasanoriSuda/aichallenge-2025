from pathlib import Path
import hashlib,json,shlex,subprocess,shutil,sys
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-peer-epochs-native-recorder-'+sys.argv[1]);out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_mpcc_architecture_snapshot.dir/link.txt').read_text())
extra=shlex.split((b/'CMakeFiles/test_mpcc_rate_resolved_certified_plan.dir/link.txt').read_text())
all_dependencies=list(dict.fromkeys(link[link.index('-o')+2:]+extra[extra.index('-o')+2:]))
archives=[x for x in all_dependencies if x.endswith('.a')]
other=[x for x in all_dependencies if not x.endswith('.a')]
sources=[p/'test/test_mpcc_architecture_snapshot.cpp',p/'src/mpcc_architecture_snapshot.cpp']
for path in [*sources,p/'include/multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp']:
    shutil.copy2(path,out/path.name)
if sys.argv[1].startswith('fixture-'):
    # Diagnostic copy only: preserve this fixture before production test cleanup.
    copy=out/'test_mpcc_architecture_snapshot.cpp'
    code=copy.read_text()
    start=code.index('TEST(MpccArchitectureSnapshot, PhysicalPlanWithoutSolverSourceKeepsCompleteObservation)')
    end=code.index('TEST(',start+5)
    block=code[start:end]
    mark='  std::filesystem::remove_all(root);'
    last=block.rindex(mark)
    block=block[:last]+'  std::filesystem::copy(root, "'+str(out/'fixture')+'", std::filesystem::copy_options::recursive);\n'+block[last:]
    copy.write_text(code[:start]+block+code[end:])
    sources[0]=copy
cmd=[link[0],'-std=c++17','-O2','-pthread','-I'+str(p/'include'),'-I/usr/include/eigen3','-I/opt/ros/humble/src/gtest_vendor/include',*map(str,sources),'-o',str(out/'regression')]+['-Wl,--start-group']+archives+['-Wl,--end-group']+other
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'regression')]+(['--gtest_filter=*PhysicalPlanWithoutSolverSource*'] if sys.argv[1].startswith('fixture-') else []),cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=60)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True);r.check_returncode()
