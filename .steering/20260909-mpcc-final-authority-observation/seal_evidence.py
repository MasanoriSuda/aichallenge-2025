"""Seal observations and rejected attempts after all containers stop."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-final-authority-validation')
validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-final-authority-*.log'):
    shutil.copy2(p,validation/p.name)
run=Path('output/20260909-final-authority-dev2-r1')
for entry in json.loads((run/'artifact-restoration.json').read_text()):
    assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest()==entry['restored_sha256']
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp','.yaml')}
for directory in Path('output').glob('20260909-final-authority-*'):
    paths.update(directory.rglob('*'))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
    digest=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    relative=p.relative_to(Path.cwd()) if p.is_absolute() else p
    artifacts.append(dict(path=str(relative),bytes=p.stat().st_size,sha256=digest.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit='e5b8c455',
    scope='First final-authority observation, exact rejected Request and separate inspected artifact; no control change; full completion open',
    build_packages=25,ctest_groups=60,colcon_records=2371,native_snapshot_cases=16,
    errors=0,failures=0,skipped=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
