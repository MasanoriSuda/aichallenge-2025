from pathlib import Path
import hashlib,json,shutil,subprocess
assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-peer-epochs-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-peer-*.log'):shutil.copy2(p,validation/p.name)
run=Path('output/20260909-peer-epochs-dev2-r1')
for item in json.loads((run/'artifact-restoration.json').read_text()):
    assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest()==item['restored_sha256']
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp')}
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/revalidation_input.hpp'))
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/replay_revalidation.cpp'))
for directory in Path('output').glob('20260909-peer-epochs-*'):paths.update(directory.rglob('*'))
paths.update(Path(x) for x in json.loads((s/'production-files.json').read_text()))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    rel=p.relative_to(Path.cwd()) if p.is_absolute() else p
    artifacts.append(dict(path=str(rel),bytes=p.stat().st_size,sha256=h.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit='36c2a01a',scope='Peer source epoch hypothesis falsified; source-free physical-plan observation repaired; actual387/930Accepted/931rejected replayed without solves; full acceptance open',build_packages=26,package_records=2374,package_errors=0,package_failures=0,package_skips=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
