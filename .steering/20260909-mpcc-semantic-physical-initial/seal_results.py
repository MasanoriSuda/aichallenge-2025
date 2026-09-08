"""Seal local initial-state acceptance and its explicit coupled limits."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

root=Path.cwd()
steering=Path(__file__).resolve().parent
single=Path('output/20260909-semantic-initial-single-r1')
after=Path('output/20260909-semantic-initial-after')
for kind in ['build','package']:
    shutil.copy2('/tmp/mpcc-20260909-semantic-initial-separated-'+kind+'.log',after/(kind+'.log'))
shutil.copy2('/tmp/mpcc-20260909-semantic-initial-replay-after.log',after/'replay-console.log')
manifest=json.loads((single/'manifest.json').read_text())
patch=subprocess.check_output(['git','diff','--','aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros'])
assert hashlib.sha256(patch).hexdigest()==manifest['working_patch_sha256'], 'Production changed after dynamic seal'

paths=set()
for directory in [Path('output/20260909-semantic-initial-before'),
                  Path('output/20260909-semantic-initial-rejected-overwrite'),
                  Path('output/20260909-semantic-initial-separated-focused'),after]:
    paths.update(p for p in directory.rglob('*') if p.is_file())
paths.update(single.glob('*.json'))
paths.update(single.glob('*.log'))
paths.update(single.glob('*.py'))
paths.update(single.glob('*.patch'))
paths.update((single/'d1').glob('*result*.json'))
paths.add(single/'d1/autoware.log')
paths.update((single/'d1/rosbag2_autoware').glob('*'))
paths.update(steering.glob('*.py'))
artifacts=[]
for path in sorted(paths):
    if not path.is_file(): continue
    digest=hashlib.sha256()
    with path.open('rb') as src:
        for chunk in iter(lambda:src.read(4*1024*1024),b''): digest.update(chunk)
    artifacts.append(dict(path=str(path.relative_to(root) if path.is_absolute() else path),
                          bytes=path.stat().st_size,sha256=digest.hexdigest()))
evidence=dict(baseline_commit=manifest['baseline_commit'],local_result='accepted',
              integrated_result='incomplete',single_run_id=single.name,
              production_patch_sha256=manifest['working_patch_sha256'],artifacts=artifacts)
(steering/'evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')

registry_path=Path('docs/spec/mpcc-experiment-registry.json')
registry=json.loads(registry_path.read_text())
snapshot_id='20260909-semantic-initial-single-r1'
if not any(s['snapshot_id']==snapshot_id for s in registry['snapshots']):
    registry['snapshots'].append(dict(snapshot_id=snapshot_id,
                                      manifest=str(steering.relative_to(root)/'evidence.json'),replay_ready=False))
source_id='20260909-peer-envelope-d1-source-396'
experiments=[
 ('overwrite-affine-x0','overwriting affine state0can preserve numerical and physical certificates',
  'physical_initial_state_representation','rejected',
  '35of2338testrecords fail numerical progress-dynamics checks; replacing one state breaks the raw affine residual contract',
  'removed; raw affine array is retained', ['affine-state0overwrite'],
  'a complete alternative numerical certificate, without relaxing its accepted residual contract'),
 ('separate-semantic-x0','a required source-bound physical seed fixes the raw-primal course-boundary failure',
  'physical_initial_state_producer','accepted',
  '132focusedtests,25packages,60CTestgroups/2339records pass; original941gets past physicalstage0; one six-lap run253.944031s/penalty0/active movingoverride0/callback18.913ms/0overrun/receipt35.070181ms',
  'physical replay/retained/connector/Stop use required exact semantic state; raw QP unchanged; integrated acceptance open',
  ['raw affine state0as physical integration origin'],
  'new failing seed/provenance/physical replay evidence'),
 ('full-circle-stop-witness','a complete-body same-world Stop is physically feasible',
  'stop_support_and_objective_comparison','accepted',
  '396world historical-objective Stop with full physical corner support passes native wall/dynamic/terminal proof; fingerprint9052539256127508737; current zero-objective Stop fails; no global infeasibility claim',
  'none; observation-only; preserve exact witness and isolate support/conditioning before any promotion',[],
  'same fixed QP or captured witness permits an independent conditioning/support comparison'),
 ('world-physical-sqp','bounded outer SQP restores the complete-body396normal candidate',
  'convexification_schedule','rejected',
  'persistent candidate still fails the first dynamic QP; overtake-only population is inapplicable toCruise; no general physical infeasibility claim',
  'none; observation-only',[],
  'a different physically supported initial convex problem or a proved numerical-conditioning defect'),
]
for suffix,hypothesis,dimension,result,reason,impact,deleted,revisit in experiments:
    item=dict(experiment_id='20260909-semantic-initial-'+suffix,
              baseline_commit=evidence['baseline_commit'],snapshot_ids=[source_id,snapshot_id],
              hypothesis=hypothesis,changed_dimension=dimension,result=result,reason=reason,
              production_impact=impact,deleted_code=deleted,revisit_condition=revisit)
    index=next((i for i,e in enumerate(registry['experiments']) if e['experiment_id']==item['experiment_id']),None)
    if index is None: registry['experiments'].append(item)
    else: registry['experiments'][index]=item
registry_path.write_text(json.dumps(registry,indent=2)+'\n')
print(json.dumps(dict(sealed_artifacts=len(artifacts),production_patch_unchanged=True,
                     registry_snapshots=len(registry['snapshots']),registry_experiments=len(registry['experiments']))))
