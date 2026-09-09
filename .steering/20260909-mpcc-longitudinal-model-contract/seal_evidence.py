"""Seal generated evidence without committing runtime artifacts."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

assert not subprocess.check_output(['docker', 'ps', '-q']).strip()
steering = Path(__file__).resolve().parent
validation = Path('output/20260909-longitudinal-validation')
validation.mkdir(exist_ok=True)
for path in Path('/tmp').glob('mpcc-longitudinal-*.log'):
    shutil.copy2(path, validation/path.name)
run = Path('output/20260909-longitudinal-physics-observed-dev2-r1')
for item in json.loads((run/'artifact-restoration.json').read_text()):
    assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['restored_sha256']
proof = json.loads(Path('output/20260909-longitudinal-physics-instrumentation-r1/cil-validation-r2.json').read_text())
assert not proof['differences'] and len(proof['probe_calls']) == 4
assert hashlib.sha256(Path(proof['original']).read_bytes()).hexdigest() == proof['original_sha256']
assert not Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/ObservationProbe.dll').exists()
assert not subprocess.check_output(['git', 'diff', '1c4f377e', '--', 'aichallenge/workspace/src/aichallenge_submit'])
paths = {path for path in steering.iterdir() if path.suffix in ('.py', '.cpp', '.hpp', '.cs', '.sh')}
for directory in Path('output').glob('20260909-longitudinal-*'):
    paths.update(directory.rglob('*'))
paths.update([
    Path('output/20260909-actuation-observed-response-r2/d1.json'),
    Path('output/20260909-actuation-observed-response-r2/d2.json'),
    Path('output/20260909-actuation-observed-dev2-r2/unity-player.log'),
])
artifacts = []
for path in sorted(path for path in paths if path.is_file()):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    relative = path.relative_to(Path.cwd()) if path.is_absolute() else path
    artifacts.append(dict(path=str(relative), bytes=path.stat().st_size, sha256=digest.hexdigest()))
evidence = dict(
    baseline_commit=subprocess.check_output(['git', 'rev-parse', 'ae2efe6a^{commit}'], text=True).strip(),
    control_baseline_commit=subprocess.check_output(['git', 'rev-parse', '1c4f377e^{commit}'], text=True).strip(),
    scope='Flat-response rest rejection, measured-state-conditioned model comparison, bounded runtime physics observations and concrete completion plan. No production repair or full MPCC acceptance.',
    authority=False, new_model_selected=False, new_package_build_or_tests=False,
    physics_receivers=2, physics_wheel_records=8, original_binary_restored=True,
    artifacts=artifacts)
(steering/'evidence.json').write_text(json.dumps(evidence, indent=2)+'\n')
print('Sealed', len(artifacts), 'artifacts')
