"""Build and exercise the new geometry lineage regressions before repair."""
from pathlib import Path
import json
import subprocess

build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out = Path('/output/20260909-coordinate-lineage-before')
out.mkdir(exist_ok=False)
cases = {
    'test_mpcc_architecture_snapshot': 'MpccArchitectureSnapshot.PreservesPublishedArtifactAndIndependentClocks',
    'test_mpcc_rate_resolved_certified_plan': 'MpccRateResolvedCertifiedPlan.RejectsCoordinateFrameChangedAfterSolve',
}
with (out/'build.log').open('w') as log:
    subprocess.run(['cmake', '--build', str(build), '--target', *cases, '-j', '4'],
                   stdout=log, stderr=subprocess.STDOUT, check=True)
results = []
for program, case in cases.items():
    with (out/(program+'.log')).open('w') as log:
        result = subprocess.run([str(build/program), '--gtest_filter='+case],
                                cwd=build, stdout=log, stderr=subprocess.STDOUT)
    results.append(dict(test=case, return_code=result.returncode))
(out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results))
