from pathlib import Path
import hashlib,json,shutil,subprocess
assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-peer-viability-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-peer-viability-*.log'):shutil.copy2(p,validation/p.name)
run=Path('output/20260909-peer-viability-dev2-r1')
for item in json.loads((run/'artifact-restoration.json').read_text()):
 assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest()==item['restored_sha256']
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp')}
paths.add(Path('.steering/20260909-mpcc-stop-proof-provenance/compare_missions.cpp'))
for directory in Path('output').glob('20260909-peer-viability-*'):paths.update(directory.rglob('*'))
for directory in Path('output/20260909-exact-join-dev2-r1/d1/mpcc_architecture_snapshots').glob('000000000941*'):paths.update(directory.rglob('*'))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
 h=hashlib.sha256()
 with p.open('rb') as stream:
  for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
 rel=p.relative_to(Path.cwd()) if p.is_absolute() else p
 artifacts.append(dict(path=str(rel),bytes=p.stat().st_size,sha256=h.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit='9d76066c',scope='941comparisons; atomic previous-accepted request observation; exact934/935zero-solve replay and peer-only counterfactual; dev2rejected/full completion open',build_packages=25,package_records=2372,package_errors=0,package_failures=0,package_skips=0,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
