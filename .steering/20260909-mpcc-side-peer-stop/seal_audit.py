"""Preserve accepted and rejected offline arms without implying runtime acceptance."""
from pathlib import Path
import hashlib
import json
import shutil
import yaml

steering=Path(__file__).resolve().parent
root=Path.cwd()
out=Path('output/20260909-side-peer-stop-audit')
source=next((out/'zero/mpcc_architecture_snapshots').glob('*/snapshot.yaml'))
shutil.copy2('/tmp/mpcc-side-peer-zero-stop-before.log',out/'native-regression-before.log')
paths={p for p in out.rglob('*') if p.is_file()}
paths.update(steering.glob('*.cpp'))
artifacts=[]
for p in sorted(paths):
    digest=hashlib.sha256(p.read_bytes()).hexdigest()
    artifacts.append(dict(path=str(p.relative_to(root) if p.is_absolute() else p),bytes=p.stat().st_size,sha256=digest))
evidence=dict(baseline_commit='b530883997e80cd78aa7e70bcd53bf2b6bcf93fc',
              source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              scope='offline architecture comparison, no runtime promotion; subsequent production repair incomplete',
              artifacts=artifacts)
(steering/'evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
registry_path=Path('docs/spec/mpcc-experiment-registry.json')
registry=json.loads(registry_path.read_text())
snapshot_id='20260909-side-peer-zero-stop-396'
if not any(s['snapshot_id']==snapshot_id for s in registry['snapshots']):
    registry['snapshots'].append(dict(snapshot_id=snapshot_id,manifest=str(steering.relative_to(root)/'evidence.json'),replay_ready=True))
arms=[
 ('kkt-only','internal equilibration fixes the saved zero Stop','numerical_policy','rejected',
  'warm solve still reaches4000iterations under both policies; cold25/50solves but dynamic proof rejects',
  'a different mathematically equivalent numerical representation or a proved coordinate defect'),
 ('common-world-only','common Cartesian geometry alone resolves the zero Stop','constraint_coordinates','rejected',
  'the exact common-frame plane corrects up to0.327006m relative-position error; all4zero-objective solves still reject',
  'a numerical representation that solves this same feasible problem'),
 ('equality-objective-only','an equivalent equality residual objective suffices for safety','numerical_representation','rejected',
  'all4solve, but original obstacle geometry fails exact dynamic proof through8bounded SQP steps',
  'corrected physical obstacle coordinate/heading linearization'),
 ('world-equality-stop','same-world geometry and equivalent feasibility representation admit safe Stop','coordinate_and_numerical_representation','accepted',
  'all4first QPs solve75/100iterations and pass native solved-Stop wall/dynamic/rest/certified-plan checks; clearance0.006051..0.008527m; exact hard constraints unchanged',
  'native production implementation, regression or additional sealed source requires validation'),
]
for suffix,hypothesis,dimension,result,reason,revisit in arms:
    item=dict(experiment_id='20260909-side-peer-'+suffix,baseline_commit=evidence['baseline_commit'],
              snapshot_ids=[snapshot_id,'20260909-peer-envelope-d1-source-396'],hypothesis=hypothesis,
              changed_dimension=dimension,result=result,reason=reason,
              production_impact='none from the comparison; evidenced producer repair and integrated acceptance remain open',
              deleted_code=[],revisit_condition=revisit)
    if not any(e['experiment_id']==item['experiment_id'] for e in registry['experiments']):
        registry['experiments'].append(item)
registry_path.write_text(json.dumps(registry,indent=2)+'\n')
print(json.dumps(dict(artifacts=len(artifacts),snapshots=len(registry['snapshots']),experiments=len(registry['experiments']))))
