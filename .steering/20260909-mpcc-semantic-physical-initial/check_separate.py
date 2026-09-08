"""Validate separate numerical and physical initial-state ownership."""
from pathlib import Path
import json
import subprocess

build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260909-semantic-initial-separated-focused')
out.mkdir(exist_ok=False)
tests = ['test_mpcc_rate_resolved_shadow','test_mpcc_rate_resolved_physical_adapter',
         'test_mpcc_rate_resolved_certified_plan','test_mpcc_architecture_snapshot']
with (out/'build.log').open('w') as log:
    subprocess.run(['cmake','--build',str(build),'--target',*tests,'-j','4'],
                   stdout=log,stderr=subprocess.STDOUT,check=True)
results=[]
for name in tests:
    with (out/(name+'.log')).open('w') as log:
        result=subprocess.run([str(build/name)],cwd=build,stdout=log,stderr=subprocess.STDOUT)
    results.append(dict(test=name,return_code=result.returncode))
    print(json.dumps(results[-1]),flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
