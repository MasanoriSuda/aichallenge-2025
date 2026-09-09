"""Register force observations and the subsequent measured-heading producer repair."""
from pathlib import Path
import json
import subprocess

path = Path('docs/spec/mpcc-experiment-registry.json')
data = json.loads(path.read_text())
baseline = subprocess.check_output(['git', 'rev-parse', '094941b5^{commit}'], text=True).strip()
force = [
 ('cil', 'Generated force observation preserves original method bodies', 'accepted', 'Seven removed calls;3608normalized methods including branch/EH targets;zero differences. Initial build failures retained; no timing parity.', 'Changed probe/binary or missing observations'),
 ('single-dev2', 'Actual force and state records expose input-to-body response', 'accepted', 'Single1lap44.6353302s/0penalty;dev2firstD1Emergency928. Horizontal wheel sum agrees with body-force delta within0.000541N; vertical/contact reaction incomplete.', 'Changed plant or explicit missing force/epoch observation'),
 ('uniform-contact', 'All four wheels are continuously and equally grounded', 'rejected', 'All16contact patterns observed. Force returns for airborne wheel. Current wire-as-net-speed model contradicted.', 'A different verified contact regime'),
 ('body-moving', 'Causal lateral/yaw body propagation improves held-out moving prediction', 'accepted', 'Initial private state and prescribed applied wire/tire inputs only;training16..35s,holdout35..59.54s;0.5spositionMAE0.19014->0.05376m. Independent dev2 improves. No future state/contact fitted.', 'New independent inputs or deployable initialization/application model'),
 ('body-promotion', 'The moving diagnostic model can execute as canonical authority', 'inconclusive', 'Private initial state and future applied actuator values are not deployable. Rest/contact/application uncertainty and full shared proof/schema remain open;1spositionMAE0.18060m.', 'Complete ROS source/history, rest/contact contract and native/replay/dynamic acceptance'),
 ('steering', 'Post-receiver steering queue and slew are reproducible without future tire feedback', 'accepted', 'Single10321steps maximum1.90735e-6degree error;dev2each3283. Applied-input boundary;unknown initial queue excluded0.3s. No DDS/application bound claimed.', 'Changed actuator or end-to-end published-input comparison'),
 ('raw-state', 'Unmodified Odometry and VelocityReport.heading_rate supply all physical body states', 'rejected', 'Odometry lateral velocity zero and filtered speed/yaw differ;raw heading_rate Euler winding spikes. Raw COM vx/vy and calibrated IMU angular velocity match physics.', 'Valid source-aligned body observation and initialization repair'),
 ('928-architecture', 'Existing A/B/C/D or Stop candidate architecture resolves force-run928', 'inconclusive', 'Same fp5606420468864100205;all9normal/4Stopsolver reject. Unknown, no physical infeasibility certificate.', 'Changed model or root-cause-supported candidate structure'),
]
heading = [
 ('counterexample', 'Declared simulation extrinsic and raceline yaw represent measured initial vehicle attitude', 'rejected', '1124source-paired physics/IMU samples require+pi/2,old-pi/2givespierror. Old path heading differs from actual starting orientation;old lever arm displaces body pose.', 'Producer repair with source/frame and native/node evidence'),
 ('producer', 'Same-source transformed attitude supplies GNSS lever arm and measured initialization', 'accepted', 'Simulation+pi/2;exactepoch pair;missingTF/invalidattitudefailclosed;source-preserving service/sharedstartup. Raw-IMU fallback removed. Vehicle chain unchanged. Generated URDF validated.', 'Changed simulator/sensor frames or startup contract'),
 ('validation', 'Changed producer preserves native and recorded-node behavior contracts', 'accepted', '26build;GNSS39records0fail7wrapper skips,initializer11/MPCC2381pass. Finalactualnode281/281outputs;poses/initialization exactr2/r4parity. Existing directcppcheck style remains;failed fixture/launch/license attempts retained.', 'Changed source, contract or unresolved test failure'),
 ('single', 'Measured-heading source completes standard uninstrumented single race', 'accepted', 'Single-r2sixlaps251.15843200683594s/penalty0/no movingoverride;maxcallback18.406ms/0overruns. Runbuildr3;laterfixstatus-onlymove verified by native and pose-exactr4replay, not a finalsameHEADcampaign.', 'Changed controller/model or required final fixed campaign'),
 ('dev2', 'Measured-heading repair completes the coupled race', 'rejected', 'FirstD1movingEmergency933/source10.309999769/v1.8082220948765995;artifact382/fp18084597137145989003. No finish. Old933/artifact371is separate.', 'Causal repair of remaining model/current-world viability failure'),
 ('933-revalidation', 'Peer-only observation change explains932accepted to933rejected', 'rejected', 'Zero-solve artifact382terminalclearance+0.00239983282409 to-0.0000303259375765m. Peer-onlycrosssubstitutionpreservesoutcomes. Controloriginspeed1.911526vsartifact2.217185m/s;independentStoprejects.', 'New exact current-world producer or model evidence'),
 ('933-architecture', 'Existing candidate architecture resolves measured-heading933', 'inconclusive', 'Nine normal A/B/C/D and four physical Stop arms all solver-reject; Unknown. No relaxed safety or physical infeasibility conclusion.', 'Changed shared model or root-cause-supported architecture'),
 ('heading-effect', 'Measured pose repair removes the large pre-start yaw discrepancy', 'accepted', 'Moving D1yawMAEagainstcalibratedIMU0.285484->0.002084rad;D2.143613->.005350. Different closedloopwindows;not independent physics truthnewruns. Filtered vx discrepancy remains.', 'Changed source/state estimator or new independent physics observation'),
]
for short, folder, rows in [('force-step','mpcc-force-step-model',force), ('measured-heading','mpcc-measured-initial-heading',heading)]:
    prefix = '20260909-'+short+'-'
    manifest = '.steering/20260909-'+folder+'/evidence.json'
    for suffix, hypothesis, result, reason, revisit in rows:
        key = prefix+suffix
        assert not any(r['snapshot_id']==key for r in data['snapshots'])
        assert not any(r['experiment_id']==key for r in data['experiments'])
        data['snapshots'].append(dict(snapshot_id=key,manifest=manifest,replay_ready=True))
        data['experiments'].append(dict(experiment_id=key,baseline_commit=baseline,
          snapshot_ids=[key],hypothesis=hypothesis,changed_dimension=('Read-only force/state/actuator diagnostics' if short=='force-step' else 'Simulation source attitude and shared initialization producer'),
          result=result,reason=reason,production_impact=('No controller/model promotion' if short=='force-step' else 'Measured-heading repair only; MPCC source/model remains1c4f377e; full acceptance open'),
          deleted_code=[] if short=='force-step' else ['initializer raw-IMU fallback/subscription/cache','service source-stamp relabel'],revisit_condition=revisit))
path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
print(len(data['snapshots']), 'snapshots;', len(data['experiments']), 'experiments')
