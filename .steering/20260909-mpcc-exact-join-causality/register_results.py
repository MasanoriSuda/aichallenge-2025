from pathlib import Path
import json
p=Path('docs/spec/mpcc-experiment-registry.json');d=json.loads(p.read_text())
prefix='20260909-exact-join-'
manifest='.steering/20260909-mpcc-exact-join-causality/evidence.json'
ids={name:prefix+name for name in ['world945','gnss-native','gnss-recorded','single-r1','dev2-r1']}
for name,i in ids.items():
    assert not any(x['snapshot_id']==i for x in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=i,manifest=manifest,replay_ready=name!='gnss-native'))
rows=[
 ('predecessor','world945','Wrong predecessor or age explains945rejection','rejected','Exact previous raw command maps through sealed gain1.435to the identical serialized float32wire value;20ms source-clock age matches. Delivery/physical tracking not thereby proved.','new inconsistent publication/request boundary'),
 ('normal','world945','Current A/B/C/D/Gnormal formulations rescue945','rejected','All nine normal arms solver-reject. Same immutable world and physical constraints; no feasibility impossibility certificate.','different bounded formulation or evidenced earlier input defect'),
 ('support-stop','world945','Existing four support Stop arms rescue945','rejected','Two solver-reject; two solve but exact dynamic proof rejects. Full peer bodies and wall limits unchanged.','different bounded physical witness or earlier causal world'),
 ('rest-missions','world945','Four full-rest formulation variants rescue945','rejected','Free/max-law times inherited/zero objective: all four solver-reject. Earlier923/1629positive witnesses are not reused as945acceptance.','new bounded formulation or upstream producer evidence'),
 ('physical-controls','world945','Bounded independent stopping controls find a geometric witness','rejected','All45steering/braking/prefix profiles fail peer or wall proof from exact current Request. Independent5ms midpoint model does not verify every original QP state/progress box; no physical infeasibility claim.','different control family or causally corrected physical input'),
 ('gnss-before','gnss-native','Subthreshold samples accumulate heading and rotate extrinsics','rejected','Actual-node4of5newcasesfail: northyaw0andmapX+0.26insteadrotatedmapY; west/reverse/nodeindependencefail. Accepted-heading noise case passes.','repair cumulative anchor producer'),
 ('gnss-after','gnss-native','Node-owned accepted anchor repairs low-speed heading','accepted','All5newnativecasesand4covariancecasespass.25packages build; GNSS30records0failures/6cppcheck2.7wrapper skips; direct host cppcheck nofindings. MPCC2371records0errors/failures/skips.','new heading ownership or transform regression'),
 ('gnss-replay','gnss-recorded','Actual repaired node updates heading on original real inputs','accepted','D1 136fixes/136pairs,D2 144fixes/143pairs. Lastyaw0changes2.167245468/2.528737748; maximumposition difference0.459502947/0.495777010m. No EKF replay, stationary-yaw or integrated-cause claim.','new source-frame or localization integration evidence'),
]
for mode in ['single','dev2']:
    root=Path('output',prefix+mode+'-r1')
    run=json.loads((root/'authority-timing-summary.json').read_text())
    accepted=True;parts=[]
    for domain,v in run['domains'].items():
        result=v['details'];moving=len(v['active_moving_overrides'])
        accepted &= bool(result and result.get('finished') and result.get('lap_count',0)>=6 and result.get('penalty_count')==0 and not moving)
        parts.append(f"{domain}:finished={bool(result and result.get('finished'))},moving_override_traces={moving},callbackmax={v['callbacks']['maximum_ms']}ms/overruns={v['callbacks']['overruns']}")
    rows.append((mode+'-race',mode+'-r1','Fixed '+mode+' race acceptance after GNSS heading repair','accepted' if accepted else 'rejected','; '.join(parts)+'. Source/receipt clocks and broader intent coverage are separate acceptance evidence.','new causal production change or recorded diagnostic need'))
    timing=[];clean=True
    for domain in run['domains']:
        topics=json.loads((root/(domain+'-topic-timing.json')).read_text())['topics']
        c=topics['/control/command/control_cmd'];o=topics['/localization/kinematic_state']
        clean &= c['source_duplicates_or_backward']==0 and c['source_gaps_over50ms']==0 and o['receive_gaps_over50ms']==0
        timing.append(f"{domain}:commandreceipt{c['receive_hz']:.6f}Hz/max{c['receive_gap_max_ms']:.6f}ms; source duplicates/backward{c['source_duplicates_or_backward']},gaps>50ms{c['source_gaps_over50ms']}; odometryreceiptmax{o['receive_gap_max_ms']:.6f}ms")
    rows.append((mode+'-delivery',mode+'-r1','Source/observation delivery is continuous','accepted' if clean else 'rejected','; '.join(timing)+'. Separate from heading correctness.','paired clock/receipt/CPU diagnosis or relevant delivery repair'))
rows.append(('replay941','dev2-r1','New first moving941failure reproduces from exact observed state','accepted','Both terminal/final941worldfp5417572569565392666record actual/inspected370and exact Request. Zero solves: steering reachable, terminalpeer-0.001206851953m, independentStoppeer-0.000951979315m; original370wall/dynamic pass. Coupled acceptance stays rejected.','bounded comparison or exact prior viable observation'))
for suffix,name,hypothesis,result,reason,revisit in rows:
    i=prefix+suffix;assert not any(x['experiment_id']==i for x in d['experiments'])
    d['experiments'].append(dict(experiment_id=i,baseline_commit='466bba5e',snapshot_ids=[ids[name]],hypothesis=hypothesis,
        changed_dimension='Named immutable-world comparison, exact source evidence or GNSS cumulative heading producer',result=result,reason=reason,
        production_impact='GNSS heading anchor ownership only; no MPCC solver/control/proof limit change; full acceptance open',
        deleted_code=['function-static shared previous position','unconditional reset of subthreshold heading baseline'] if suffix=='gnss-after' else [],revisit_condition=revisit))
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(len(d['snapshots']),'snapshots;',len(d['experiments']),'experiments')
