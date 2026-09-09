from pathlib import Path
import hashlib,json,subprocess,sys,shutil
s=Path(__file__).resolve().parent;p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
out=Path('/output/20260909-stop-viability-native-epoch-'+sys.argv[1]);out.mkdir(exist_ok=False)
sources=[p/'test/test_mpc_state_prediction.cpp' if sys.argv[1].startswith('package-') else s/'reproduce_epoch.cpp',p/'src/mpc_state_prediction.cpp',p/'src/mpc_longitudinal_prediction.cpp']
for f in sources:shutil.copy2(f,out/f.name)
cmd=['g++','-std=c++17','-O2','-pthread','-I'+str(p/'include'),'-I/opt/ros/humble/src/gtest_vendor/include',*map(str,sources),'-lgtest_main','-lgtest','-o',str(out/'regression')]
with (out/'build.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'regression')],stdout=log,stderr=subprocess.STDOUT,timeout=30)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,sources={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},meaning='Real native prediction/observer and exact application call sequence; no ROS node or race claim; baseline expected to fail nonzero observation-age invariants.'),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True);r.check_returncode()
