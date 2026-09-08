"""Append positive, rejected and inconclusive stopping-proof observations."""
from pathlib import Path
import json

path = Path('docs/spec/mpcc-experiment-registry.json')
data = json.loads(path.read_text())
manifest = '.steering/20260909-mpcc-complete-stop-suffix/evidence.json'
names = ['native', 'free-artifact', 'free-multicycle', 'single-r1', 'world4264-executed3695']
ids = ['20260909-complete-stop-' + name for name in names]
for name in ids:
    assert not any(s['snapshot_id'] == name for s in data['snapshots'])
    data['snapshots'].append(dict(snapshot_id=name, manifest=manifest,
        replay_ready=name != ids[0]))
world968 = '20260909-rear-peer-dev2-world968-executed354'
entries = [
 ('968-initial-pose',[world968], 'certified successor starts at actual current pose', 'rejected',
  'Native original bundle displaced37.925041mm/yaw-0.002027228rad. Correct current-world reproof rejects wall; original positive successor was a false witness.',
  'exact projection-frame ownership repair'),
 ('pose-producer',[ids[0],world968], 'one owned reference coordinate preserves current body', 'accepted',
  'Native before translates150mm; repair removes stale iteration and derived bundle coordinate. Correct968pose error0and static-path-blocked. No new availability claim.',
  'new frame/body round-trip mismatch'),
 ('free-before',[ids[0],ids[1]], 'old terminal consumer preserves feasible complete stopping suffix', 'rejected',
  'Full current-world wall/peer continuation passes; separately generated maximum brake rejects d1at-0.006402223m. Independent rear-peer native witness also rejects.',
  'current-state complete-suffix terminal ownership'),
 ('complete-terminal',[ids[0],ids[1]], 'same complete current-state trajectory can prove terminal rest', 'accepted',
  '67retained cases/60CTestgroups/2366records/25packages pass. Free candidate current-world and command adapter pass, dynamic min2.491691m. Speed perturbations reject old planned-rest reuse. Free producer remains offline.',
  'new physical or identity mismatch'),
 ('initial-multicycle',[ids[2]], 'initial repeated-publication helper completes through rest', 'inconclusive',
  'Helper stopped with publisher interval mismatch before summary; native fixture then proves35msmetadata versus25msclock under0.016physical tolerance. Binary/input/log preserved.',
  'exact command-clock metadata correction'),
 ('publisher-clock',[ids[0],ids[2]], 'terminal metadata owns exactly one publisher interval', 'accepted',
  'Native before/after and conditional replay verify exact25ms; physical tolerance no longer determines the new suffix command count. Existing physical margins unchanged.',
  'new command-boundary mismatch'),
 ('conditional188',[ids[2]], 'one frozen source supports repeated current-model joins through rest', 'accepted',
  '188joins and command conversions accepted;170solved suffix/18existing generated terminal proofs.130msdelay retained. CV/model observations, interpolated early current speed and fixed steering diagnostic; no future sensor truth/live publication.',
  'actual async/runtime adoption or changed candidate producer'),
 ('single-r1',[ids[3]], 'consumer repairs preserve single six-lap acceptance', 'accepted',
  '6laps253.323883s/penalty0, moving override0.10413reportedcycles/12.918msmax/0overruns. Command receiptmax34.149647ms; source gapmax35ms/no duplicates.',
  'new production change'),
 ('dev2-r1',[ids[4]], 'consumer repairs establish coupled race acceptance', 'rejected',
  'D1first4264at8.72m/s terminal wall reject13, independent correct-pose Stop wall reject. Actual3695captured. No finish;3564/3573reportedcycles max20.292/20.403ms/0overruns. Candidate generation remains unresolved.',
  'demonstrated current-world candidate producer change or required bounded causal replay'),
]
for suffix, snapshots, hypothesis, result, reason, revisit in entries:
    name = '20260909-complete-stop-' + suffix
    assert not any(e['experiment_id'] == name for e in data['experiments'])
    data['experiments'].append(dict(experiment_id=name, baseline_commit='8d6ebf8a',
        snapshot_ids=snapshots, hypothesis=hypothesis,
        changed_dimension='named Stop pose/terminal consumer invariant or conditional offline replay',
        result=result, reason=reason,
        production_impact='current physical pose and terminal proof preserved; free candidate producer offline; full acceptance open',
        deleted_code=['stale Stop projection iteration and derived bundle coordinate'] if suffix == 'pose-producer' else
                     ['unconditional replacement of already proved complete stopping suffix'] if suffix == 'complete-terminal' else [],
        revisit_condition=revisit))
path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
print(len(data['snapshots']), 'snapshots;', len(data['experiments']), 'experiments')
