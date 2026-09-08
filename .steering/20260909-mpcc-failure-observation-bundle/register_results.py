"""Append this slice's bounded outcomes once, retaining earlier failed scenes."""
from pathlib import Path
import json

path=Path('docs/spec/mpcc-experiment-registry.json')
data=json.loads(path.read_text())
manifest='.steering/20260909-mpcc-failure-observation-bundle/evidence.json'
snapshots=[('20260909-failure-bundle-native',False),('20260909-failure-bundle-world1629-executed1100',True)]
for name,ready in snapshots:
    assert not any(s['snapshot_id']==name for s in data['snapshots'])
    data['snapshots'].append(dict(snapshot_id=name,manifest=manifest,replay_ready=ready))
native=snapshots[0][0];current=snapshots[1][0];old='20260909-longitudinal-dev2-world4428'
entries=[
('4428-architectures',[old],'bounded A/B/C/D and fresh Stop find certified alternative','unchanged world/model/hard requirements','rejected',
 'Nine normal arms bundle0; A/B/Gleft terminal wall contact107 at6.448m; C/D/Gright dynamic QP reject. Fresh Stop input ends before rest, no arms attempted. Physical feasibility Unknown.',
 'new candidate architecture or a demonstrated producer correction'),
('independent-dedup',[native],'independent bounded world and source records preserve every written failure join','native old-library fixture, two distinct worlds/artifacts','rejected',
 'Second world Written but its source Duplicate under generic publication bucket. Missing actual3834cannot be reconstructed.',
 'atomic ownership of world/source/artifact/clock'),
('atomic-observation',[native,current],'one bounded transaction preserves exact publication for each written world','observation writer and its call sites only','accepted',
 'Native14/source104,25package build,60CTestgroups/2362records pass; dev2first1629world contains actual1100execution. No new authority.',
 'new observation mismatch or changed recorder boundary'),
('dev2-r1',[current],'unchanged control completes two-vehicle six laps','observation-only runtime diagnostic','rejected',
 'D2decision1629at4.342925m/s terminal dynamic d1 failure; D1subsequent1678Emergency. No finished result-details. Each1178reportedcycles/0overruns.',
 'demonstrated control producer fix or a newly required bounded observation'),
('1629-zero-solve',[current],'actual published1100current-world terminal failure is exactly reproducible','recorded controls and clock, native diagnostic copy only','accepted',
 'Original horizon certifies; retained track min-0.0067710926225m at1.555s (573checks), normal-0.0061931006377m at1.56s. Delay/continuation clear; terminal wall not reached.',
 'same captured input with a causally justified candidate architecture'),
('1629-architectures',[current],'existing normal and Stop alternatives cover rear-only Cruise scene','same sealed1629world and full peer radius','rejected',
 'Astage19linearization unavailable; B/C/D/Gfront target unavailable. Four Stop solves all dynamic reject d1near1.50409s. No physical infeasibility proof.',
 'rear-peer-aware independent candidate construction or demonstrated linearization repair'),
('held-out-peer',[current],'replacing CV future by actual peer future explains complete terminal rejection','future GNSS scoring only; unchanged full body circle','rejected',
 'First rejected instant becomes clear, but full hypothetical Stop still overlaps by0.096/0.103m. D1future includes its later Emergency; predictor-only root cause not established.',
 'causal peer prediction evidence before intervention, or new physical trajectory candidate'),
]
for suffix,ids,hypothesis,dimension,result,reason,revisit in entries:
    name='20260909-failure-bundle-'+suffix
    assert not any(e['experiment_id']==name for e in data['experiments'])
    data['experiments'].append(dict(experiment_id=name,
        baseline_commit='6c4875ed4d250a44cd813935b234863e9c184701',snapshot_ids=ids,
        hypothesis=hypothesis,changed_dimension=dimension,result=result,reason=reason,
        production_impact='observation only; integrated MPCC acceptance remains open',
        deleted_code=['independent production world/source recorder calls'] if suffix=='atomic-observation' else [],
        revisit_condition=revisit))
path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
print(len(data['snapshots']),'snapshots;',len(data['experiments']),'experiments')
