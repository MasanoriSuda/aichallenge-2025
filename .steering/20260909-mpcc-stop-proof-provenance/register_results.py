"""Append this completed slice without erasing earlier accepted/rejected trials."""
from pathlib import Path
import json
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-stop-provenance-'
manifest='.steering/20260909-mpcc-stop-proof-provenance/evidence.json'
ids={name:prefix+name for name in ['native','world923-actual328','world1629','world4264','single-r1','dev2-r1']}
for name,i in ids.items():
    assert not any(s['snapshot_id']==i for s in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=i,manifest=manifest,replay_ready=name!='native'))
rows=[
 ('metadata-before',['native'],'Stop proof binds the solved artifact residual','rejected','Valid observation baseline4.2e-5 differs from solved artifact: new native fails before repair with obstacle empty/minimum inf. Other four focused cases pass.','post-solve certificate producer repair'),
 ('metadata-after',['native'],'Correct producer closes the provenance mismatch without loosening validator','accepted','Five focused Stop cases pass;25-package build and60CTestgroups/2369scoped records pass. Strict mismatch tests retained.','new proof ownership failure'),
 ('probe',['world923-actual328'],'Frozen2056/2162/923explain live invalid dynamic proof','inconclusive','All three paired source-copy probes reject at solve before proof; cannot attribute the live failure to metadata from these observations.','captured solved Stop artifact or exact source-validation reason'),
 ('actual328',['world923-actual328'],'Recorded328original proof can be reproduced without a replacement solve','accepted','Zero solves: original wall/dynamic pass, min1.182361692m; terminal4.166666642m/s is not rest. First helper link failure preserved; r2 fixes only linker dependency. Current923steering and actual395artifact remain missing.','new current physical observation or actual publication evidence'),
 ('923-normal',['world923-actual328'],'A/B/C/D/Gnormal candidates rescue923','rejected','Nine normal arms reject; rough-right C solves then fails wall proof. Remaining normal arms reject at solver.','different bounded candidate/formulation or earlier causal world'),
 ('923-stop-support',['world923-actual328'],'Existing Stop comparisons contain a physical witness','accepted','Two inherited-objective maximum-braking arms accept; two zero-objective arms fail physical wall. CLI historical title does not make their braking controls free. No current-state join or runtime publication.','exact accepted candidate source/artifact comparison'),
 ('923-missions',['world923-actual328'],'Shared-clock/all-peer Stop formulations cover923','accepted','Of four explicit offline candidates, maximum-braking/inherited-objective passes full proofs: first a-2.959595971m/s2, minpeer0.664353915m. Free inherited/feasibility solve-reject; maximum feasibility wall-reject. No current-state join or publication.','evidenced production candidate architecture or newly captured current observation'),
 ('1629-missions',['world1629'],'Shared-clock/all-peer free controls preserve a stopping witness','accepted','Inherited and zero-objective free controls pass; both maximum-law candidates solver-reject. Zero-objective free first a-0.659250633m/s2, minpeer1.836994342m. All candidate identities/artifacts retained; no runtime promotion.','new production candidate or actual async observation'),
 ('4264-missions',['world4264'],'The same four Stop formulations rescue4264','rejected','All four reject at solver or exact wall proof. Scene feasibility remains Unknown.','different bounded physical witness or earlier causal observation'),
]
for mode in ['single','dev2']:
    root=Path('output',prefix+mode+'-r1')
    run=json.loads((root/'authority-timing-summary.json').read_text())
    domains=run['domains'];accepted=True;parts=[]
    for domain,v in domains.items():
        detail=v['details'];moving=len(v['active_moving_overrides'])
        ok=bool(detail and detail.get('finished') and detail.get('lap_count',0)>=6 and detail.get('penalty_count')==0 and not moving)
        accepted=accepted and ok
        parts.append(f"{domain}:finished={bool(detail and detail.get('finished'))},moving_override_traces={moving},callbackmax={v['callbacks']['maximum_ms']}ms/overruns={v['callbacks']['overruns']}")
    rows.append((mode+'-r1',[mode+'-r1'],'Certificate ownership repair preserves fixed '+mode+' race acceptance','accepted' if accepted else 'rejected','; '.join(parts)+'. Throttled trace counts are observations; details and timing remain in the sealed run.','new production change or demonstrated cause of the recorded failure'))
    if mode=='single':
        timing=json.loads((root/'d1-topic-timing.json').read_text())['topics']
        c=timing['/control/command/control_cmd'];o=timing['/localization/kinematic_state'];clock=timing['/clock']
        clean=c['source_duplicates_or_backward']==0 and c['source_gaps_over50ms']==0 and o['receive_gaps_over50ms']==0
        rows.append(('single-clock',['single-r1'],'Finished single has continuous source/observation delivery','accepted' if clean else 'rejected',f"Command receipt{c['receive_hz']:.6f}Hz/max{c['receive_gap_max_ms']:.6f}ms; source duplicates/backward{c['source_duplicates_or_backward']},gaps>50ms={c['source_gaps_over50ms']},max{c['source_gap_max_ms']:.6f}ms. Odometry receiptmax{o['receive_gap_max_ms']:.6f}ms; clock receiptmax{clock['receive_gap_max_ms']:.6f}ms. No moving override; delivery cause remains unproven.",'new paired source/receipt/CPU observation or relevant delivery repair'))
for suffix,names,hypothesis,result,reason,revisit in rows:
    i=prefix+suffix;assert not any(e['experiment_id']==i for e in d['experiments'])
    d['experiments'].append(dict(experiment_id=i,baseline_commit='ae6862aa',snapshot_ids=[ids[n] for n in names],hypothesis=hypothesis,changed_dimension='named certificate producer repair, sealed offline formulation or fixed runtime observation',result=result,reason=reason,production_impact='Only solved-certificate producer metadata and source-validation diagnostic change; offline formulations have no publication authority; full acceptance remains open',deleted_code=['pre-solve ReplayWorld tolerance copied into solved Stop physical certificate'] if suffix=='metadata-after' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
