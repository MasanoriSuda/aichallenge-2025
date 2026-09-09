from pathlib import Path
import json,subprocess
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-stop-viability-';manifest='.steering/20260909-mpcc-stop-viability-epochs/evidence.json'
baseline=subprocess.check_output(['git','rev-parse','4b2474da^{commit}'],text=True).strip()
ids={n:prefix+n for n in ['old931','native-epoch','single-r1','dev2-r1']}
for key in ids.values():
    assert not any(x['snapshot_id']==key for x in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=key,manifest=manifest,replay_ready=True))
rows=[
 ('peer-clock','old931','Peer field or publication mapping alone explains old387/930to931','rejected','Both counterfactual substitutions preserve Accepted930 and rejected931; same peer170/CVfield. Anchor difference about1ns, not old4.4ms defect.','new causal peer or clock evidence'),
 ('normal','old931','Bounded A/B/C/D/G candidates rescue frozen931','rejected','All9normal methods solver-reject with model/hard limits unchanged; physical feasibility Unknown.','new formulation witness or causal input repair'),
 ('support-stop','old931','Existing supportStop candidates rescue frozen931','rejected','All4supportStop methods solver-reject; no physical infeasibility certificate.','new bounded candidate or causal producer'),
 ('native-before','native-epoch','Raw old-epoch state reaches its claimed future control epoch','rejected','Before-r2:zero-age2pass,5/15msage4fail. Straight2m/s omits10/30mm; curved yaw omits0.00075/0.00225rad. Before-r1 separately retains6.09nm midpoint quadrature false failure.','changed state-time producer'),
 ('repair','native-epoch','Explicit source-to-now estimation preserves state epoch and raw monitoring','accepted','34native,104source contracts,26package build,60CTestgroups/2381records0errors/failures/skips. First ROSPose2Dconstructor compile failure preserved then corrected. Canonical current-pose owner separated; async copy and strict prefix checks maintained.','new epoch/owner regression'),
 ('held-twist','old931','Source-to-now held-twist estimate rescues old930/931','rejected','Counterfactual source shifts19.149526/6.564913mm; both terminal-reject -0.000858296754/-0.002501689556m. Old acceptance may mask omitted motion; no actual-motion accuracy claim.','independent measurement/model evidence'),
 ('single','single-r1','Fresh single six-lap acceptance','accepted','254.0740509033203s/penalty0/no moving override; callbackmax15.327ms/0overruns. Command/Odometry/clock source and receipts have no >50msgap or source duplicates/backward.','relevant production change or new regression'),
 ('runtime-epoch','dev2-r1','Current pose matches the declared source-to-now estimator in runtime','accepted','927/928/930uniquely match same-run raw Odometry projected19.999999/15/5ms: XY/yawerror0. Shifts25.892985/19.419744/6.812562mm. This is implementation validation, not ground-truth motion accuracy.','new unmatched observation'),
 ('race','dev2-r1','State epoch repair establishes coupled acceptance','rejected','First movingD1Emergency930at1.362512395068054m/s; no finish. Normal374terminal928rejects;386Stopjoins; actual374publishes929. Finalinspected386wallpasses, terminalpeer -0.0013934640174382285m and independentStop -0.0011199987734020755m reject. Actual/inspected clocks must not be mixed.','causal actual-command/application or current-world proof evidence'),
 ('timing','dev2-r1','Control and delivery timing acceptance is complete','inconclusive','D1/D2callbackmax20.969/23.696ms and0overruns; commandreceipt~40Hz/no >50msgap. Source12duplicates each, gaps2/1; Odometry/clock delivery pauses~300ms. No causal-observation warnings.','causal delivery/application phase evidence'),
]
for suffix,name,hypothesis,result,reason,revisit in rows:
    key=prefix+suffix;assert not any(x['experiment_id']==key for x in d['experiments'])
    d['experiments'].append(dict(experiment_id=key,baseline_commit=baseline,snapshot_ids=[ids[name]],hypothesis=hypothesis,changed_dimension='Odometry source-to-now constant-twist estimation before unchanged actuator prediction; current-pose owner',result=result,reason=reason,production_impact='No authority/gains/actuator delay/model/hard-limit/solver changes; state estimate explicitly timestamped; full acceptance open',deleted_code=['raw old-epoch state fed directly to now-based actuator rollout','canonical proof prefix-start bound to raw wall-monitor pose'] if suffix=='repair' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
