"""Keep all before/after, architectural and runtime outcomes, including rejects."""
from pathlib import Path
import json
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-complete-rest-'
manifest='.steering/20260909-mpcc-complete-rest-candidates/evidence.json'
names=['native','world4264-comparison','world1629-retimed','production-replay','single-r1','dev2-world923-executed328']
ids=[prefix+n for n in names]
for i in ids:
 assert not any(s['snapshot_id']==i for s in d['snapshots'])
 d['snapshots'].append(dict(snapshot_id=i,manifest=manifest,replay_ready=i!=ids[0]))
rows=[
 ('actual3695',[ids[1]],'published artifact is preserved without re-solving','accepted','Original wall/dynamic pass; current4264delay/publisher clear but continuation/terminal wall reject. Exact Stop initial pose error0.','new measured world/executed artifact'),
 ('initial-before',[ids[0],ids[1]],'initial soft reference always lies in nonlinear frame domain','rejected','Native13mm overshoot fails;4264reference transition overshoots12.779200mm. A/B/C/D/G and free-rest all fail before solve.','shared initial and iterated tangent-domain selection'),
 ('initial-after',[ids[0],ids[1]],'one tangent-domain selector preserves physical model and original objective','accepted','All20adapter cases pass; same4264can now reach solve, which still rejects wall QP. No raw iterate or hard-bound relaxation.','new domain/reference mismatch'),
 ('4264-retimed',[ids[1]],'bounded complete-rest timings rescue4264','rejected','4.543438906s and5s x maximum-law/free-controls x depth0/3 all reject wall proof or wall QP. Original1.780641s cannot brake8.709218m/s torest atmin-3. Scene feasibility remains Unknown.','new physical candidate/witness or earlier causal observation'),
 ('1629-retimed',[ids[2]],'free controls through rest can preserve rear-peer clearance','accepted','5s maximum-law0/3reject rear QP; free0/3accept exact wall, peer, rest and current command join. All outcomes retained.','changed production generation or real async observation'),
 ('producer',[ids[0],ids[3]],'production current-world candidate can own complete-rest/all-peer identity','accepted','Old native rear-peer failure-0.0110606m; new native and invalid clock/rest/secondary guards pass. Short wall-profile domain regression fixed without editing acceptance.25build/60groups/2368local records pass.','new source/geometry/peer/clock mismatch'),
 ('production-replay',[ids[3]],'new production candidate joins the same measured world','accepted','1629one candidate35.444582ms, full join/command available, min2.433887479m.4264still wall-QP rejected125.019816ms; no runtime publication in replay.','actual async/runtime or changed candidate'),
 ('single-r1',[ids[4]],'new producer preserves single6lap acceptance','accepted','6laps252.778778s/penalty0/moving override0;10375reportedcycles max18.373ms/0overruns;10398commands40.001061Hz receiptmax39.216995ms/no gaps>50ms.','new production change'),
 ('async-publication',[ids[5]],'certified candidate crosses actual async publication boundary','accepted','D1decision844/source308at0.02m/s and D2decision827/source309at0.05m/s published certified normal commands. Does not establish moving multi-tick Stop/restart.923alternate395joined/selected alone is not publication proof.','moving Stop continuation or new identity concern'),
 ('dev2-r1',[ids[5]],'new producer establishes coupled race acceptance','rejected','First visible moving Emergency D2decision925at1.823493m/s. Atomic first terminal world923preserves normal328; not925world nor395Stop artifact. D1max31.967ms/3overruns, D2max22.277ms/0. No lap/finish; commands40Hz/no gaps>50ms.','bounded923/328replay, exact switching chronology and demonstrated causal change'),
]
for suffix,snapshots,hypothesis,result,reason,revisit in rows:
 i=prefix+suffix;assert not any(e['experiment_id']==i for e in d['experiments'])
 d['experiments'].append(dict(experiment_id=i,baseline_commit='3c1cd03b',snapshot_ids=snapshots,hypothesis=hypothesis,changed_dimension='named tangent domain, complete-rest candidate/clock/all-peer architecture or fixed runtime campaign',result=result,reason=reason,production_impact='current-world maximum-law producer replaced; sole certified authority preserved; full acceptance open',deleted_code=['current-world fixed maximum-braking candidate and secondary lattice execution path'] if suffix=='producer' else ['separate incomplete initial tangent selection'] if suffix=='initial-after' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
