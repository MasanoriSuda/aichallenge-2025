"""Seal shared-model diagnostics and their exact local inputs without promotion."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import numpy as np

ROOT = Path('.steering/20260910-mpcc-shared-plant-model')
VALIDATION = Path('output/20260910-shared-plant-validation-r1')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


assert not subprocess.check_output(['docker', 'ps', '-q']).strip()
assert not subprocess.check_output(['git', 'diff', '1c4f377e', '--',
    'aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'])
assert not Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/ObservationProbe.dll').exists()
protected = {
    'aichallenge/result-summary.json': '170d447ece696f257ae7542903caa15a6d04762ec54847f2a43b3b1b29adddd1',
    'aichallenge/safety-gate-result.json': 'a297f288d5e9bc39f241c41d9b6b39891ccaabec76a594cf088a6b766fc2038e',
    'aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll':
        '703e18fad4e3cf68111a559190edb7060e901988a04c409d84c80331dd45a172',
}
for name, expected in protected.items():
    assert digest(Path(name)) == expected, name
gear_validation = {}
for name in ['single', 'dev2']:
    raw = np.load(f'output/20260909-force-step-analysis-{name}-r1/physics.npz')
    index = {str(key): i for i, key in enumerate(raw['columns'])}
    gear = raw['values'][:, index['gear']]
    assert np.all(gear == 3), 'Drive-only component used outside Drive'
    gear_validation[name] = dict(records=len(gear), drive_records=int(np.sum(gear == 3)))
VALIDATION.mkdir(exist_ok=True)
for source in Path('/tmp').glob('mpcc-shared-plant-*.log'):
    shutil.copy2(source, VALIDATION/source.name)
paths = {p for p in ROOT.iterdir() if p.suffix in ['.py', '.cpp', '.hpp']}
for directory in Path('output').glob('20260910-shared-plant-*'):
    paths.update(p for p in directory.rglob('*') if p.is_file())
inputs = [
    'output/20260909-force-step-body-models-r1/report.json',
    'output/20260909-force-step-analysis-single-r1/physics.npz',
    'output/20260909-force-step-analysis-dev2-r1/physics.npz',
    'output/20260909-force-step-ros-state-single-r1/d1.json',
    'output/20260909-force-step-ros-state-dev2-r1/d1.json',
    'output/20260909-force-step-ros-state-dev2-r1/d2.json',
    'output/20260909-initial-heading-node-replay-r4/d1/report.json',
    'output/20260909-initial-heading-node-replay-r4/d2/report.json',
    'output/20260908-mpcc-stop-reference-single-r3/unity-collider-geometry.json',
    'output/20260908-mpcc-stop-reference-single-r3/unity-kart1-tree.json',
    'output/20260909-longitudinal-model-inputs-r1/unity-wheel-cil.txt',
    'output/20260909-actuation-response-r1/unity-plant-cil.txt',
    'aichallenge/simulator/AWSIM/AWSIM_Data/level1',
    'aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/Vehicle/vehicle.yaml',
    'aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll',
]
paths.update(Path(p) for p in inputs)
artifacts = [dict(path=str(p), bytes=p.stat().st_size, sha256=digest(p)) for p in sorted(paths)]
report = dict(
    baseline_commit=subprocess.check_output(['git', 'rev-parse', '9bca3af6^{commit}'], text=True).strip(),
    control_baseline_commit=subprocess.check_output(['git', 'rev-parse', '1c4f377e^{commit}'], text=True).strip(),
    new_model_authority=False, full_mpcc_acceptance=False,
    acceptance_requirement_question_pending=True, drive_precondition_validation=gear_validation,
    protected_user_and_simulator_artifacts=protected, artifacts=artifacts)
(ROOT/'evidence.json').write_text(json.dumps(report, indent=2)+'\n')
print(len(artifacts), 'artifacts sealed; Drive-only precondition:', gear_validation, flush=True)
