from pathlib import Path
import hashlib,json,shlex,subprocess
s=Path(__file__).resolve().parent;p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros');b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-actuation-native-model-r1');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
a=list(dict.fromkeys(x for x in link if x.endswith('.a') and 'gtest' not in x));deps=[x for x in link[link.index('-o')+2:] if not x.endswith('.a')]
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'native_longitudinal.cpp'),'-o',str(out/'check'),'-Wl,--start-group',*a,'-Wl,--end-group',*deps]
with (out/'build.log').open('w') as log:built=subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
built.check_returncode()
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'check')],cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=30)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,build_return_code=built.returncode,native_return_code=r.returncode,source_sha256=hashlib.sha256((s/'native_longitudinal.cpp').read_bytes()).hexdigest(),authority=False,meaning='Current canonical v derivative treats wire acceleration as net acceleration. Analytic isolated decoded rolling resistance component yields two74mm/s mismatches; no complete plant identification or production repair.'),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True)
assert r.returncode==1,'Expected old-model mismatch was not reproduced'
