from pathlib import Path
import json,subprocess
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-actuation-';manifest='.steering/20260909-mpcc-actuation-application/evidence.json'
baseline=subprocess.check_output(['git','rev-parse','1c4f377e^{commit}'],text=True).strip()
ids={name:prefix+name for name in ['original930','instrumentation','observed-r1','observed-r2','native-model']}
for name,key in ids.items():
 assert not any(x['snapshot_id']==key for x in d['snapshots'])
 d['snapshots'].append(dict(snapshot_id=key,manifest=manifest,replay_ready=name!='observed-r1'))
rows=[
 ('published374','original930','Actual374 with its929 published clock rescues original930 world','rejected','Diagnostic assembly,zero solves: steering-unreachable; terminal -0.0025014656576010097m and independentStop -0.0011199987734020755m. Original inspected386 preserved. Two helper input-schema failures preserved before validr3.','new causal state/model/command evidence'),
 ('cil','instrumentation','Four generated-only probes preserve original decoded control flow','accepted','3608methods compare equal after removing Receive1/Choose1/Apply2calls, including branch/EH targets. Initial token/NaN comparison failure and disposable-container dpkg failure preserved. No runtime timing equivalence claim.','probe or original binary change'),
 ('missing-log','observed-r1','Direct application phase captured in first diagnostic run','inconclusive','FirstD1Emergency938; Unity defaultPlayer.log not captured. No application-time claim forr1.','explicit persisted UnityPlayer.log; exercised inr2'),
 ('selection','observed-r2','Received and applied identities can be observed without participant authority changes','accepted','1906receives/438applications; all438selected sequence/source-ns/float32input joins match; probeerrors0. Receiver/Domain mapping from581/609unique wire keys. OriginalDLL and protectedJSON unchanged.','new mismatch or instrumentation change'),
 ('fixed-delay','observed-r2','Every published acceleration is applied exactly0.13s later','rejected','D1/D2received946/960,applied217/221,overwritten729/739; selected-source-age median30.650449/25.096941ms and max103.655550ms. Application period max183.330866ms, not a bound.','different verified receiver/application contract'),
 ('missed-stop','observed-r2','A missed certifiedStop pulse is necessary to explain first933Emergency','rejected','Before933no moving brake publication. FirstEmergency packet430at9.834999780 and431overwritten; packet432applies9.923955841. First request to brake88.956061ms. Original374/386run phase remains unknown.','different exact-run evidence; never transfer this phase'),
 ('wire-net','observed-r2','Constant applied wire acceleration equals body velocity derivative','rejected','9.309999791..9.799999780s raw15velocity samples gain0.365482807m/s versus wire integration0.651501815m/s; actual input1.32959557..1.32959855m/s². Not a full plant fit.','causal unified longitudinal model or independent calibration'),
 ('native-before','native-model','Canonical seven-state transition represents known rolling-resistance component','rejected','Native current library,straight forwardv2m/s,wire+1/-1,dt.2: both74mm/s velocity mismatches against isolated decodedrollingR.37. Pre-repair failures,not package-test failures or full-plant accuracy proof.','shared wire/net model repair'),
 ('peer-only','observed-r2','Peer-field substitution alone removes371932to933viability transition','rejected','Exactzero-solve932Accepted+0.0019944623102432502m vs933rejected-0.0013267385265398612m. Cross-substituting peer field projectedtoeachtime preserves both outcomes.','new peer/source evidence'),
 ('normal','observed-r2','Current A/B/C/D/G normal methods rescue fixed933world','rejected','All9normal methods solver-reject with unchanged model/hardconstraints. Physical feasibility Unknown.','new causal model/producer or bounded witness'),
 ('support-stop','observed-r2','Existing supportStop methods rescue fixed933world','rejected','All4supportStop methods solver-reject. No physical infeasibility certificate.','new causal model/producer or bounded witness'),
]
for suffix,name,hypothesis,result,reason,revisit in rows:
 key=prefix+suffix;assert not any(x['experiment_id']==key for x in d['experiments'])
 d['experiments'].append(dict(experiment_id=key,baseline_commit=baseline,snapshot_ids=[ids[name]],hypothesis=hypothesis,changed_dimension='Diagnostic receiver/application observation and native model comparison; participant source unchanged',result=result,reason=reason,production_impact='No participant authority/model/gain/delay/solver/limit change. Generated probe DLL only in isolated diagnostic mount,removedafterrun. Full acceptance open.',deleted_code=[],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
