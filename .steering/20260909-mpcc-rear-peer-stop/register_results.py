"""Append all bounded outcomes, preserving previous rejected scenes."""
from pathlib import Path
import json

path = Path('docs/spec/mpcc-experiment-registry.json')
data = json.loads(path.read_text())
manifest = '.steering/20260909-mpcc-rear-peer-stop/evidence.json'
names = ['native', 'world1629', 'single-r1', 'dev2-world968-executed354']
ids = ['20260909-rear-peer-' + name for name in names]
for name in ids:
    assert not any(s['snapshot_id'] == name for s in data['snapshots'])
    data['snapshots'].append(dict(snapshot_id=name, manifest=manifest,
                                 replay_ready='world' in name))
entries = [
 ('profile164',1,'lateral references make maximum-brake terminal feasible','rejected',
  'All164complete terminal paths reject rear d1; no physical infeasibility claim.',
  'different longitudinal/terminal architecture'),
 ('tangent-domain',0,'selected tangent transition stays in immutable model domain','accepted',
  'Old native regression fails;19native/2364package records/25build packages pass after tangent-only intersection. Raw primal/hard bounds unchanged.',
  'new tangent-domain violation'),
 ('same-world-normal',1,'tangent repair closes full rear-peer failure','rejected',
  'A solves and horizon certifies, terminal dynamic minimum-0.000656495m. Other normal arms lack front target; four Stop methods reject.',
  'terminal candidate architecture change'),
 ('rear-qp8',1,'rear peer QP guidance closes full stopping proof','rejected',
  'Eight methods reject terminal/physical/QP proof; none promoted.',
  'changed terminal control family'),
 ('free-controls4',1,'complete free controls through rest admit a physical witness','accepted',
  'Automatic0/3 and StayAhead3 accepted, StayAhead0 assembly rejected;3/4. Full wall/peer/rest proof unchanged. Offline only, no publication artifact claim.',
  'exact artifact/current-world multicycle publication audit'),
 ('single-r1',2,'tangent repair preserves single six-lap acceptance','accepted',
  '6laps253.328887939s/penalty0, moving override0;10423reportedcycles max12.671ms/0overruns. Receiptmax33.745289ms. Source timing anomalies retained.',
  'new production change'),
 ('dev2-r1',3,'tangent repair establishes coupled acceptance','rejected',
  'D2decision968at1.762816m/s terminal wall reject33; dynamic clear+0.725731m. Actual354captured. Stop435steering join rejected; independently certified successor bundle439terminal reproof rejected. No finish. Each365cycles/0overruns.',
  'demonstrated current-world terminal producer repair'),
]
for suffix,index,hypothesis,result,reason,revisit in entries:
    name = '20260909-rear-peer-' + suffix
    assert not any(e['experiment_id'] == name for e in data['experiments'])
    data['experiments'].append(dict(experiment_id=name, baseline_commit='4afa9948',
        snapshot_ids=[ids[index]], hypothesis=hypothesis,
        changed_dimension='tangent-domain repair or named offline terminal architecture; see sealed per-method manifest',
        result=result, reason=reason, production_impact='tangent producer only; coupled acceptance remains open',
        deleted_code=['independent-only progress/input tangent selection'] if suffix == 'tangent-domain' else [],
        revisit_condition=revisit))
path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
print(len(data['snapshots']), 'snapshots;', len(data['experiments']), 'experiments')
