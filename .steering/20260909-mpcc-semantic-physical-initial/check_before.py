"""Run existing solve and feedback tests with strengthened physical x0 contract."""
from pathlib import Path
import json
import subprocess

build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260909-semantic-initial-before')
out.mkdir(exist_ok=False)
with (out/'build.log').open('w') as log:
    subprocess.run(['cmake','--build',str(build),'--target','test_mpcc_rate_resolved_shadow','-j','4'],
                   stdout=log,stderr=subprocess.STDOUT,check=True)
with (out/'test.log').open('w') as log:
    result = subprocess.run([str(build/'test_mpcc_rate_resolved_shadow'),
                             '--gtest_filter=MpccRateResolvedShadow.SolvesAndSamplesOnePublicationInterval:MpccRateResolvedShadow.LatestStateFeedback*'],
                            cwd=build,stdout=log,stderr=subprocess.STDOUT)
(out/'result.json').write_text(json.dumps(dict(return_code=result.returncode))+'\n')
print('regression_return_code='+str(result.returncode),flush=True)
