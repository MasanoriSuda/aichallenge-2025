from pathlib import Path
import json
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-peer-viability-';manifest='.steering/20260909-mpcc-peer-viability/evidence.json'
ids={k:prefix+k for k in ['world941','native','dev2-r1']}
for name,i in ids.items():
 assert not any(x['snapshot_id']==i for x in d['snapshots'])
 d['snapshots'].append(dict(snapshot_id=i,manifest=manifest,replay_ready=name!='native'))
rows=[
('normal','world941','A/B/C/D/Gnormal candidates rescue941','rejected','All nine arms solver-reject. Same world and hard constraints. No impossibility certificate.','different bounded formulation or evidenced earlier input defect'),
('support-stop','world941','Existing supportStop candidates rescue941','rejected','Two solver-reject;two exactdynamicreject with firstoverlap-0.00034656m.','different bounded witness or causal input defect'),
('rest-missions','world941','Four explicit full-rest formulations rescue941','rejected','Free/inherited and free/zero solver-reject; max-law/inherited and max-law/zero exactpeer-reject-0.000325794/-0.000325568m. Unknown feasibility.','new formulation or causal input evidence'),
('covariance-units','world941','Shadow covariance_m2name identifies a consumer unit defect','rejected','Wire V2XVehiclePosition defines standard deviations metres. Current radius sum uses wire values consistently; sqrtwouldbeincorrect.','actual producer/wire contract change or contradictory numeric evidence'),
('observer','native','Previous ordinary acceptance survives and is atomically paired with failure','accepted','17native snapshot cases,104source contracts,25package build and60CTestgroups/2372records0errors/failures/skips. Initial fixture compiletypeerror preserved; final namedCellvalues pass package. No control/proof change.','new missing or inconsistent observed boundary'),
('paired-replay','dev2-r1','Exact same376input reproduces934Accepted to935terminalrejection','accepted','934obs9.854999779terminal+0.017855530645m;935obs9.869999779terminal-0.000355659220m. Zero solves;935independentStopstillaccepted+0.007945880181m. No publication claim.','ego/clock/command causal comparison on these exact inputs'),
('peer-causality','dev2-r1','The peer update alone flips934to935acceptance','rejected','Reprojecting the other observed CVpeerfield to each unchanged ego clock preserves both outcomes:934accepted+0.028189770500m,935rejected-0.000997344491m. Counterfactual only, no authority.','different paired scene or interaction with separately evidenced ego change'),
('race','dev2-r1','Fixed dev2race acceptance','rejected','FirstmovingD1Emergency938v1.34m/s;no finish/lap. CallbackD1/D2max21.762/19.321ms,0overruns.','causal production change or recorded diagnostic need'),
('delivery','dev2-r1','Source/observation delivery is continuous','rejected','40Hzcommands and0receiptgap>50ms; source12duplicate/backwardand2gapseach. Odometryreceiptpauses340.522/333.068ms,clock331.446/327.150ms.','paired timing/CPU/input investigation or relevant delivery repair'),
('final388','dev2-r1','Final938inspectedStop388has complete solver-source replay evidence','inconclusive','Actualpublication376and exact rejected Request retained; inspectedplan388solver source missing. PriorordinaryAccepted937/376 is explicitly invalid as same-artifact predecessor. Do not invent388from376.','recover actual derived artifact and derivation provenance without substitution')]
for suffix,name,hyp,result,reason,revisit in rows:
 i=prefix+suffix;assert not any(x['experiment_id']==i for x in d['experiments'])
 d['experiments'].append(dict(experiment_id=i,baseline_commit='9d76066c',snapshot_ids=[ids[name]],hypothesis=hyp,
 changed_dimension='Named immutable-world comparison or exact previous ordinary observation',result=result,reason=reason,
 production_impact='Observation only; no solver/control/proof limit change; full acceptance open',
 deleted_code=['accepted Request discard','ShiftOut/Pass-only accepted boundary coverage'] if suffix=='observer' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
