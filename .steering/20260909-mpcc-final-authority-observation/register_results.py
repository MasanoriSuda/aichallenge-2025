"""Preserve each observation hypothesis, including rejected coupled acceptance."""
from pathlib import Path
import json
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-final-authority-'
manifest='.steering/20260909-mpcc-final-authority-observation/evidence.json'
ids={name:prefix+name for name in ['native','actual365','dev2-r1','world945-actual361','world1137-actual575']}
for name,i in ids.items():
    assert not any(s['snapshot_id']==i for s in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=i,manifest=manifest,replay_ready=name!='native'))
rows=[
 ('old-scheduling','native','Accepted terminal submission guarantees final observation','rejected','Native existing recorder deduplicates second terminal; explicit final writes. Blocked LatestOnlyWorker drops first accepted pending final job. Controller used that submission to suppress final recording.','first-event observer repair'),
 ('new-scheduling','native','First boundary ownership survives a busy writer','accepted','16native cases/104source contracts,25-package build/60CTestgroups/2371records pass; pending final preserved, shutdown drains, invalid supplemental evidence retains valid world.','new ownership or serialization defect'),
 ('actual365','actual365','Original published365replays without a substitute solve','accepted','Original wall/dynamic pass with zero solves/minpeer0.2508901557m/terminal3.5688619844m/s. Missing old951/379cannot be reconstructed.','new exact old failure evidence'),
 ('capture945','world945-actual361','First moving final failure has exact world, publication and inspected Request','accepted','D1first moving Emergency945/v1.5802064876844846. Atomic worldfp7865628471136372292/published361/inspected361/exact Request all present. Both Domain callbackoverruns0.','new missing boundary or final selection path'),
 ('replay945','world945-actual361','Exact captured Request reproduces rejection without solving','accepted','Zero-solve current945/361reproduces steering-unreachable, terminalpeer-0.003647424709m and independent Stop peer-0.001124094799m; original source wall/dynamic pass. Helper r1link failure retained; r2only adds libraries. Control cause remains unresolved.','paired publisher timeline or bounded architecture comparison'),
 ('later1137','world1137-actual575','Later observed terminal rejection remains reproducible','accepted','1137/575exact Request replays terminal wall rejection27with zero solves. Occurs after first945Emergency but before explicit teardown; not an earliest-failure replacement.','causal need to inspect downstream behavior'),
 ('race','dev2-r1','Fixed dev2 satisfies race acceptance','rejected','No finish/lap; D1moving Emergency945. D1/D2callbackmax18.836/21.067ms and0overruns. Commands40Hz/no receiptgap>50ms.','new causal production change'),
 ('delivery','dev2-r1','Command source and odometry delivery are continuous','rejected','D1/D2source command duplicates/backward14/12,gaps>50ms3/2,max80/85ms. Odometry receiptmax305.603504/309.693813ms; clockmax307.422876/306.540012ms. No causal attribution to945without paired timeline.','paired source/receipt/publisher observation'),
]
for suffix,name,hypothesis,result,reason,revisit in rows:
    i=prefix+suffix;assert not any(e['experiment_id']==i for e in d['experiments'])
    d['experiments'].append(dict(experiment_id=i,baseline_commit='e5b8c455',snapshot_ids=[ids[name]],
        hypothesis=hypothesis,changed_dimension='First-event observation owner or exact captured evidence only',
        result=result,reason=reason,production_impact='Observation only; no solver/control/proof/publisher change; full acceptance open',
        deleted_code=['terminal submission suppresses final authority record','LatestOnlyWorker replacement for first-failure observation'] if suffix=='new-scheduling' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
