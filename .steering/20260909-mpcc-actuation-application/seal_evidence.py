from pathlib import Path
import hashlib,json,shutil,subprocess
assert not subprocess.check_output(['docker','ps','-q']).strip()
s=Path(__file__).resolve().parent
validation=Path('output/20260909-actuation-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-actuation-*.log'):shutil.copy2(p,validation/p.name)
run=Path('output/20260909-actuation-observed-dev2-r2')
for item in json.loads((run/'artifact-restoration.json').read_text()):
 assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest()==item['restored_sha256']
probe=Path('output/20260909-actuation-instrumentation-r2')
proof=json.loads((probe/'cil-validation-r2.json').read_text())
assert not proof['differences']
assert hashlib.sha256(Path(proof['original']).read_bytes()).hexdigest()==proof['original_sha256']
assert not Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/ObservationProbe.dll').exists()
paths={p for p in s.iterdir() if p.suffix in ('.py','.cpp','.hpp','.cs','.sh')}
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/revalidation_input.hpp'))
paths.add(Path('.steering/20260909-mpcc-publication-clock-handoff/replay_revalidation.cpp'))
for directory in Path('output').glob('20260909-actuation-*'):paths.update(directory.rglob('*'))
artifacts=[]
for p in sorted(p for p in paths if p.is_file()):
 h=hashlib.sha256()
 with p.open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 rel=p.relative_to(Path.cwd()) if p.is_absolute() else p
 artifacts.append(dict(path=str(rel),bytes=p.stat().st_size,sha256=h.hexdigest()))
(s/'evidence.json').write_text(json.dumps(dict(baseline_commit=subprocess.check_output(['git','rev-parse','1c4f377e^{commit}'],text=True).strip(),scope='Direct received/selected/applied input observation. No production repair. Native wire/net rolling-component discrepancy, exact371932/933 and original374/930diagnostics, architectureUnknown, updated full completion plan.',cil_methods_compared=3608,inserted_probe_calls=4,received=1906,applied=438,application_identity_mismatches=0,native_pre_repair_mismatches=2,artifacts=artifacts),indent=2)+'\n')
print('Sealed',len(artifacts),'artifacts')
