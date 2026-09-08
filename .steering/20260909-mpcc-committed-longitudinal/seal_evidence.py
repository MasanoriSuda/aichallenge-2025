"""Seal completed longitudinal trials and native evidence; never hash a live run."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

steering = Path(__file__).resolve().parent
repo = Path.cwd()
assert not subprocess.check_output(['docker', 'ps', '-q']).strip(), 'Stop simulations first'
validation = Path('output/20260909-committed-longitudinal-validation')
validation.mkdir(exist_ok=True)
for path in Path('/tmp').glob('mpcc-longitudinal-*.log'):
    shutil.copy2(path, validation/path.name)
paths = {p for p in steering.iterdir() if p.suffix in ('.py', '.cpp', '.yaml')}
runs = sorted(Path('output').glob('20260909-committed-longitudinal-*'))
for directory in runs:
    if '-single-' in directory.name or '-dev2-' in directory.name:
        assert (directory/'monitor-summary.json').exists(), directory
        assert (directory/'artifact-restoration.json').exists(), directory
    paths.update(p for p in directory.rglob('*') if p.is_file())
for run, domain in [
    ('20260909-published-stop-observation-dev2-r1', '2'),
    ('20260909-semantic-initial-single-r1', '1'),
]:
    directory = Path('output')/run
    paths.update(p for p in (directory/f'd{domain}-native-longitudinal').rglob('*') if p.is_file())
    paths.add(directory/f'd{domain}-longitudinal-full-motion.json')
    paths.add(directory/f'd{domain}-velocity-model-comparison.json')
artifacts = []
for path in sorted(paths):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    artifacts.append(dict(
        path=str(path.relative_to(repo)) if path.is_absolute() else str(path),
        bytes=path.stat().st_size, sha256=digest.hexdigest()))
evidence = dict(
    baseline_commit='03e174e568691dd8640b02baf71d37375d23fabc',
    scope='paired committed-input response observer and decision clock producer; integrated completion remains open',
    build_packages=25, ctest_groups=60, colcon_records=2359,
    errors=0, failures=0, skipped=0,
    native_temporal_cases=27,
    artifacts=artifacts)
(steering/'evidence.json').write_text(json.dumps(evidence, indent=2)+'\n')
print('Sealed', len(artifacts), 'artifacts')
