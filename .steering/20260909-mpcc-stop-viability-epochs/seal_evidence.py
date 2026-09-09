from pathlib import Path
import hashlib,json,shutil,subprocess
assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-stop-viability-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-stop-viability-*.log'):shutil.copy2(p,validation/p.name)
run=Path('output/20260909-stop-viability-dev2-r1')
for item in json.loads((run/'artifact-restoration.json').read_text()):
    assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest()==item['restored_sha256']
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp')}
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/revalidation_input.hpp'))
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/replay_revalidation.cpp'))
paths.add(Path('.steering/20260909-mpcc-peer-observation-epochs/certified_plan_input.hpp'))
paths.add(Path('.steering/20260909-mpcc-peer-observation-epochs/replay_certified_plan.cpp'))
for directory in Path('output').glob('20260909-stop-viability-*'):paths.update(directory.rglob('*'))
paths.update(Path(x) for x in json.loads((s/'production-files.json').read_text()))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
    h=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    rel=p.relative_to(Path.cwd()) if p.is_absolute() else p
    artifacts.append(dict(path=str(rel),bytes=p.stat().st_size,sha256=h.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit='4b2474da',scope='Explicit source-to-now motion estimation and canonical current-pose ownership; native known-motion defect repaired; single accepted/dev2 rejected; live current-pose projection exactly matched; command application phase open',build_packages=26,package_records=2381,package_errors=0,package_failures=0,package_skips=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
