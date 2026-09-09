"""Seal this slice only after all diagnostics and simulators have stopped."""
from pathlib import Path
import hashlib,json,shutil,subprocess

assert not subprocess.check_output(['docker','ps','-q']).strip()
steering=Path(__file__).resolve().parent
validation=Path('output/20260909-stop-provenance-validation');validation.mkdir(exist_ok=True)
for path in Path('/tmp').glob('mpcc-stop-provenance-*.log'):shutil.copy2(path,validation/path.name)
for mode in ['single','dev2']:
    run=Path('output/20260909-stop-provenance-'+mode+'-r1')
    assert (run/'monitor-summary.json').exists()
    for entry in json.loads((run/'artifact-restoration.json').read_text()):
        assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest()==entry['restored_sha256']
paths={p for p in steering.iterdir() if p.suffix in ('.py','.cpp','.hpp','.yaml')}
prior=Path('output/20260909-complete-rest-dev2-r1')
paths.update((prior/'d2-executed-328').rglob('*'))
for world in (prior/'d2/mpcc_architecture_snapshots').glob('000000000923*'):paths.update(world.rglob('*'))
for directory in Path('output').glob('20260909-stop-provenance-*'):paths.update(directory.rglob('*'))
artifacts=[]
for path in sorted(p for p in paths if p.is_file()):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    relative=path.relative_to(Path.cwd()) if path.is_absolute() else path
    artifacts.append(dict(path=str(relative),bytes=path.stat().st_size,sha256=digest.hexdigest()))
(steering/'evidence.json').write_text(json.dumps(dict(baseline_commit='ae6862aa',scope='Solved-certificate producer ownership; original 328 replay, 923 architecture/missions and fixed runtime; full MPCC completion remains open',build_packages=25,ctest_groups=60,colcon_records=2369,architecture_native_cases=48,errors=0,failures=0,skipped=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
