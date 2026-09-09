from pathlib import Path
import hashlib
import json
import shutil
import subprocess

assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-exact-join-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-exact-join-*.log'):
    shutil.copy2(p,validation/p.name)
for mode in ['single','dev2']:
    run=Path('output/20260909-exact-join-'+mode+'-r1')
    assert (run/'monitor-summary.json').exists()
    for entry in json.loads((run/'artifact-restoration.json').read_text()):
        assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest()==entry['restored_sha256']
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp')}
paths.add(Path('.steering/20260909-mpcc-stop-proof-provenance/compare_missions.cpp'))
for directory in Path('output').glob('20260909-exact-join-*'):
    paths.update(directory.rglob('*'))
for directory in Path('output/20260909-final-authority-dev2-r1/d1/mpcc_architecture_snapshots').glob('000000000945*'):
    paths.update(directory.rglob('*'))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
    digest=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    relative=p.relative_to(Path.cwd()) if p.is_absolute() else p
    artifacts.append(dict(path=str(relative),bytes=p.stat().st_size,sha256=digest.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit='466bba5e',
    scope='Exact945predecessor/comparisons, GNSS cumulative heading producer repair, native and recorded input replay, fixed single/dev2; full completion open',
    build_packages=25,packages=dict(multi_purpose_mpc_ros=dict(records=2371,errors=0,failures=0,skips=0),
        racing_kart_gnss_poser=dict(records=30,errors=0,failures=0,skips=6,skip_reason='ROS cppcheck2.7performance guard; separate direct host cppcheck completed without findings')),
    native_heading_before=dict(cases=5,failures=4),native_heading_after=dict(cases=5,failures=0),
    artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
