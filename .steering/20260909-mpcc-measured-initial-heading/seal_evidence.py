"""Seal both dependent slices without committing generated binary/bag evidence."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


assert not subprocess.check_output(['docker','ps','-q']).strip()
assert not Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/ObservationProbe.dll').exists()
proof = json.loads(Path('output/20260909-force-step-instrumentation-r4/cil-validation-r2.json').read_text())
assert not proof['differences'] and len(proof['probe_calls']) == 7
assert digest(Path(proof['original'])) == proof['original_sha256']
assert not subprocess.check_output(['git','diff','1c4f377e','--','aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'])
protected = {'aichallenge/result-summary.json':'170d447ece696f257ae7542903caa15a6d04762ec54847f2a43b3b1b29adddd1',
 'aichallenge/safety-gate-result.json':'a297f288d5e9bc39f241c41d9b6b39891ccaabec76a594cf088a6b766fc2038e'}
for name, value in protected.items():
    assert digest(Path(name)) == value, name
for name, patterns, logs in [
 ('mpcc-force-step-model',['20260909-force-step-*'],['mpcc-force-step-*.log']),
 ('mpcc-measured-initial-heading',['20260909-initial-heading-*','20260909-measured-heading-*'],['mpcc-initial-heading-*.log','mpcc-measured-heading-*.log'])]:
    steering = Path('.steering/20260909-'+name)
    validation = Path('output/20260909-'+('force-step' if name=='mpcc-force-step-model' else 'initial-heading')+'-validation')
    validation.mkdir(exist_ok=True)
    for pattern in logs:
        for src in Path('/tmp').glob(pattern):
            shutil.copy2(src, validation/src.name)
    paths = {p for p in steering.iterdir() if p.suffix in ['.py','.cpp','.hpp','.cs','.sh']}
    for pattern in patterns:
        for directory in Path('output').glob(pattern):
            paths.update(p for p in directory.rglob('*') if p.is_file())
    if name=='mpcc-measured-initial-heading':
        paths.update(Path(p) for p in subprocess.check_output(['git','diff','--name-only','--','aichallenge/workspace/src/aichallenge_submit'], text=True).splitlines())
        paths.add(Path('aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser/test/test_gnss_imu_heading.cpp'))
    artifacts = [dict(path=str(p),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(paths)]
    report = dict(baseline_commit=subprocess.check_output(['git','rev-parse','094941b5^{commit}'],text=True).strip(),
      control_baseline_commit=subprocess.check_output(['git','rev-parse','1c4f377e^{commit}'],text=True).strip(),
      scope=name, new_model_authority=False, full_mpcc_acceptance=False,original_binary_restored=True, protected_user_artifacts=protected,artifacts=artifacts)
    (steering/'evidence.json').write_text(json.dumps(report,indent=2)+'\n')
    print(name,len(artifacts),'sealed artifacts',flush=True)
