from pathlib import Path
import json,subprocess
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-peer-epochs-';manifest='.steering/20260909-mpcc-peer-observation-epochs/evidence.json'
baseline=subprocess.check_output(['git','rev-parse','36c2a01a^{commit}'],text=True).strip()
ids={name:prefix+name for name in ['peer-motion','fixture','validation','dev2-r1']}
for key in ids.values():
    assert not any(x['snapshot_id']==key for x in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=key,manifest=manifest,replay_ready=True))
rows=[
 ('peer-time','peer-motion','Per-vehicle source time is replaced by array or receipt epoch in old928/929','rejected','Actual production filter reproduces both velocities exactly; projecting raw XY using vehicle source epoch yields zero XY error. Receipt is bag observation, not controller receipt. Filter lag is not itself a bug.','new raw source/receipt evidence'),
 ('native-before','fixture','A valid source-free CertifiedPlan retains its physical evidence','rejected','Before-r2:17old cases pass; new actual accepted wall-proof fixture fails because physical evidence is omitted. Before-r1 link failure is separately preserved.','changed recorder'),
 ('repair','validation','Additive source-free physical-plan capture preserves existing contracts','accepted','Final18native,104source cases pass;26package build and60CTestgroups/2374records0errors/failures/skips. Invalid publication isolated; solver absence explicit; no authority change.','new observation regression'),
 ('roundtrip','fixture','Complete recorded physical evidence reproduces its original proof','accepted','Source-free decoder validates context/pose/frame/grid identities; every wall diagnostic field exactly matches after reading actual fixture record, zero solves. Compile and filename-glob failures remain separate evidence.','changed schema or physical evaluator'),
 ('capture','dev2-r1','Actual derived Stop artifact/publication and accepted/rejected Requests survive','accepted','Actual387published at930 preserved with all physical evidence. Same387Accepted930/rejected931 retained independently; legacy solver-source missing remains. All three physical proof roles replay exactly with zero solves.','new missing actual-plan boundary'),
 ('transition','dev2-r1','Same387keeps terminal acceptance at931','rejected','After30ms, terminal+0.007091597519957693m becomes -0.0003133950625793247m; independent Stop+0.008026033410105438m becomes -0.0006277465960242701m. Wall continuation passes. Same peer170/CVfield and equivalent publication mapping.','causal ego/time/peer proof investigation'),
 ('normal-peer','dev2-r1','Peer-only update explains new377/927Accepted to928rejected','rejected','Counterfactual peer-only cross-substitution keeps both outcomes; anchor-only preserves928rejection. Original377wall/dynamic pass, horizon ends3.52075m/s. Independent Stop928accepts and derived387joins.','new independent causal input evidence'),
 ('race','dev2-r1','Observation repair establishes coupled six-lap acceptance','rejected','First moving D1Emergency931at1.3129826481757174m/s; no finish/details. Observation repair changes no control semantics.','causal production change or named missing observation'),
 ('timing','dev2-r1','Callback and message delivery acceptance is complete','inconclusive','D1/D2max20.668/26.215ms,0/1overrun; D2reported window after firstEmergency. Command receipt~40Hz/no >50msgap; source duplicates/backward9/10 and Odometry/clock delivery pauses remain.','causal timing evidence or producer repair'),
 ('parent-proof','dev2-r1','Original parent solver/dynamic proof is available for derived387','inconclusive','Complete actual artifact and accepted physical wall proof are available; CertifiedPlan has no parent solver snapshot or original dynamic-proof object. No fabricated source or replacement solve. Current Requests are replayable.','explicit parent derivation observation if needed for the next causal test'),
]
for suffix,name,hypothesis,result,reason,revisit in rows:
    key=prefix+suffix;assert not any(x['experiment_id']==key for x in d['experiments'])
    d['experiments'].append(dict(experiment_id=key,baseline_commit=baseline,snapshot_ids=[ids[name]],hypothesis=hypothesis,changed_dimension='Observation-only complete physical plan; named native/source epoch/exact replay comparisons',result=result,reason=reason,production_impact='Passive immutable plan observation; no authority/model/limits/gains/solver change; full acceptance open',deleted_code=['solver-source prerequisite for saving existing physical artifact/proof'] if suffix=='repair' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
