"""Seal peer validation and rejected trial, keeping raw outputs out of git."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path.cwd()
steering = Path(__file__).resolve().parent
run = Path('output/20260909-peer-envelope-dev2-r1')
validation = Path('output/20260909-peer-envelope-validation')
for src, name in [('/tmp/mpcc-20260909-peer-dev2-r1.log','monitor-console.log'),
                  ('/tmp/mpcc-20260909-peer-world-comparison.log','world-comparison-helper-failure.log'),
                  ('/tmp/mpcc-20260909-peer-world-comparison-r2.log','world-comparison-console.log')]:
    if Path(src).exists():
        shutil.copy2(src,run/name)
paths = set(validation.glob('*.log'))
paths.update(run.glob('*.json'))
paths.update(run.glob('*.log'))
paths.update(run.glob('d*/autoware.log'))
paths.update(run.glob('d*/mpcc_architecture_snapshots/*/snapshot.yaml'))
paths.update((run/'first-failure-comparison').glob('*'))
paths.update((run/'world-target-comparison-r2').rglob('*.yaml'))
paths.update((run/'world-target-comparison-r2').rglob('*.json'))
paths.update((run/'world-target-comparison-r2').rglob('*.log'))
paths.update(steering.glob('*.py'))
paths.update(steering.glob('*.cpp'))
manifest = dict(baseline_commit='5cfc50bc685d9ce52c489d7d9962e42ae43ec9c5',
                run_id=run.name, integrated_result='rejected',
                original_runtime_snapshot_count=len(list(run.glob('d*/mpcc_architecture_snapshots/*/snapshot.yaml'))),
                source_identity='baseline plus exact peer patch/source/binary hashes in run manifest',
                artifacts=[])
for path in sorted(paths):
    if path.is_file():
        manifest['artifacts'].append(dict(path=str(path.relative_to(root) if path.is_absolute() else path),
                                         bytes=path.stat().st_size,
                                         sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(steering/'evidence.json').write_text(json.dumps(manifest,indent=2)+'\n')

registry_path = Path('docs/spec/mpcc-experiment-registry.json')
registry = json.loads(registry_path.read_text())
prefix = '20260909-peer-envelope-'
snapshot_ids = [prefix+'d1-source-'+str(n) for n in [394,396,941]]
for snapshot_id in snapshot_ids:
    if not any(s['snapshot_id']==snapshot_id for s in registry['snapshots']):
        registry['snapshots'].append(dict(snapshot_id=snapshot_id,
                                          manifest=str(steering.relative_to(root)/'evidence.json'),
                                          replay_ready=True))
experiments = [
 ('enclosure','complete peer radius encloses all four measured bodies',
  'peer_body_geometry','accepted',
  'old circle reports +0.822m for actual body overlap; new native predicate rejects it; independent hull/config tests and25-package build/60CTestgroups/2338records pass',
  'shared1.876mnominal body plus unchanged uncertainty; retired ego-width subtraction and independent Recovery radius',
  ['ego half-width subtraction from peer circle','stuck_recovery.rear_safety.vehicle_radius_m'],
  'different body asset or V2X reference point with independently measured geometry'),
 ('dev2-r1','complete peer body is sufficient for coupled race acceptance',
  'integrated_validation','rejected',
  'moving Emergency/Recovery, one lap each, D1callback26.284ms/1overrun; formal domain results absent; side peer exists in proof world but optimizer has no target tube',
  'no promotion to integrated acceptance',[],
  'fixed-input producer repair and successful package/build evidence justify a new sealed trial'),
 ('original-architectures','existing candidate architectures recover first side-peer encounter',
  'architecture_comparison','inconclusive',
  'A394/396solves but native dynamic proof rejectsD2; A941physical-stage0rejects. B/C/D/Gcannot generate without canonical target tube; not evidence of physical infeasibility',
  'none; observation-only',[],
  'same-world complete target producer or proven physical-state repair with resealed identity'),
 ('world-target-architectures','same-world peer tube restores existing A/B/C/Dcandidate feasibility',
  'target_constraint_producer','rejected',
  'constant-velocity peer tube with original full circle/world/dt/hard bounds is resealed; all A/B/C/D/Garms on394/396/941reject dynamic QP rows at unchanged4000iterations; feasibility remains Unknown',
  'none; target producer is offline only',[],
  'different bounded physical homotopy/formulation, or proof of faulty geometric/time coordinate mapping'),
 ('raw-initial-native','certified raw QPx0can replace semantic physical initial state',
  'physical_initial_state','rejected',
  'source396rawtheta=-2.1804138714481612e-9rejects course sampling and first native step; semantic0with identical control/model/window accepts',
  'bounded semantic-initial producer repair follows in separate steering',[],
  'physical initial-state producer or course-window provenance changes'),
]
for suffix,hypothesis,dimension,result,reason,impact,deleted,revisit in experiments:
    item=dict(experiment_id=prefix+suffix,baseline_commit=manifest['baseline_commit'],
              snapshot_ids=snapshot_ids,hypothesis=hypothesis,changed_dimension=dimension,
              result=result,reason=reason,production_impact=impact,deleted_code=deleted,
              revisit_condition=revisit)
    old=next((i for i,e in enumerate(registry['experiments']) if e['experiment_id']==item['experiment_id']),None)
    if old is None: registry['experiments'].append(item)
    else: registry['experiments'][old]=item
registry_path.write_text(json.dumps(registry,indent=2)+'\n')
print(json.dumps({k:v for k,v in manifest.items() if k!='artifacts'}))
print('sealed_artifacts='+str(len(manifest['artifacts'])))
