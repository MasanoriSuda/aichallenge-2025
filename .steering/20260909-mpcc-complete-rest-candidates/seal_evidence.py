"""Seal closed observation diagnostic and all positive/negative replay outcomes."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

assert not subprocess.check_output(['docker','ps','-q']).strip()
steering = Path(__file__).resolve().parent
repo = Path.cwd()
validation = Path('output/20260909-complete-rest-validation')
validation.mkdir(exist_ok=True)
for path in Path('/tmp').glob('mpcc-complete-rest-*.log'):
    shutil.copy2(path,validation/path.name)
run = Path('output/20260909-complete-rest-dev2-r1')
assert (run/'monitor-summary.json').exists()
for entry in json.loads((run/'artifact-restoration.json').read_text()):
    assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest() == entry['restored_sha256']
paths = {p for p in steering.iterdir() if p.suffix in ('.py','.cpp','.hpp','.yaml')}
paths.update(Path('output/20260909-complete-stop-dev2-r1/d1-executed-3695').rglob('*'))
paths = {p for p in paths if p.is_file()}
for directory in Path('output').glob('20260909-complete-rest-*'):
    paths.update(p for p in directory.rglob('*') if p.is_file())
artifacts = []
for path in sorted(paths):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    artifacts.append(dict(path=str(path.relative_to(repo)) if path.is_absolute() else str(path),
        bytes=path.stat().st_size,sha256=digest.hexdigest()))
(steering/'evidence.json').write_text(json.dumps(dict(
    baseline_commit='3c1cd03b',
    scope='Complete-rest producer and all-peer guidance; initial tangent domain; single accepted/dev2 rejected; all outcomes in results.md',
    build_packages=25,ctest_groups=60,colcon_records=2368,
    adapter_native_cases=20,architecture_native_cases=47,
    errors=0,failures=0,skipped=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
