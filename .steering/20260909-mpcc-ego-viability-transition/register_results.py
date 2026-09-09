"""Register the bounded EKF repair and preserve rejected coupled acceptance."""
from pathlib import Path
import json

p = Path('docs/spec/mpcc-experiment-registry.json')
d = json.loads(p.read_text())
prefix = '20260909-ego-viability-'
manifest = '.steering/20260909-mpcc-ego-viability-transition/evidence.json'
ids = {k: prefix + k for k in ['old-pair', 'native', 'parity', 'single-r1', 'dev2-r1']}
for name, key in ids.items():
    assert not any(x['snapshot_id'] == key for x in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=key, manifest=manifest, replay_ready=True))
rows = [
    ('ego-sensitivity', 'old-pair', 'Named ego groups affect exact 935 terminal viability', 'accepted',
     'Previous speeds or poses/progress separately rescue the hybrid request; steering, command or clock groups do not. Counterfactuals are not measured trajectories or authority.', 'different paired scene or causal producer evidence'),
    ('native-baseline', 'native', 'Installed EKF state and output share one calculation epoch', 'rejected',
     'Real installed timer callback: fixed-clock control passes; both 5 ms mid-callback advances violate state/stored/output epoch. Six clock reads. Positive-advance cases only.', 'installed binary or clock implementation changes'),
    ('repair', 'native', 'One sampled EKF epoch removes the demonstrated producer inconsistency', 'accepted',
     'Four real-node clock tests pass, including queued measurements and observed Odometry; 23 original math tests pass. 26-package build; EKF 102 records, 0 failures, 30 cppcheck wrapper skips; MPCC 2372 records pass. One unchanged upstream Eigen cppcheck false positive retained.', 'new clock or filter regression'),
    ('parity', 'parity', 'The repair preserves filter mathematics on shared recorded input', 'accepted',
     '54 ticks, 2268 state/covariance values, maximum absolute difference 0. Shared initialization and frozen callback clocks; not original hidden-state reconstruction.', 'filter equations or configuration change'),
    ('single', 'single-r1', 'Fresh single-car six-lap acceptance', 'accepted',
     '255.779419 s, penalty 0, no observed moving override; callback maximum 15.460 ms, 0 overruns. Throttled telemetry limits remain.', 'relevant production change or new regression evidence'),
    ('race', 'dev2-r1', 'EKF repair establishes coupled race acceptance', 'rejected',
     'First moving D1 Emergency 937 at 1.3724878243982674 m/s; no finish. Callback maxima D1/D2 19.636/21.167 ms, 0 overruns.', 'causal production change or justified missing observation'),
    ('paired-replay', 'dev2-r1', 'Exact same-374 inputs reproduce Accepted 936 and rejected 937', 'accepted',
     'Zero solves, terminal clearance +0.004820238906648067 to -0.0014817573213388169 m; independent Stop +0.007846802037691836 to -0.0005679827768239054 m. Original artifact certified.', 'causal ego/clock/publication comparison'),
    ('peer-causality', 'dev2-r1', 'Peer field update alone flips 936 to 937', 'rejected',
     'Peer-only cross-substitution projected to each unchanged ego clock preserves both outcomes and reported terminal clearances. No authority.', 'different paired input or demonstrated interaction'),
    ('clock-owner', 'dev2-r1', 'The observed execution-clock anchor change is a producer defect', 'inconclusive',
     'Observation advances 30 ms; artifact cursor advances 25.596119 ms because same-source anchors differ. Ownership and publication chronology not yet audited.', 'exact source/publication clock ownership and native invariant reproduction'),
    ('delivery', 'dev2-r1', 'Observation and command source times are continuous', 'rejected',
     'Command receipts about 40 Hz with no gap over 50 ms, but source duplicates/backward and gaps remain. D1/D2 Odometry receipt maxima 277.653/282.086 ms, clock 281.079/280.086 ms.', 'causal timing investigation or relevant producer repair'),
]
for suffix, name, hypothesis, result, reason, revisit in rows:
    key = prefix + suffix
    assert not any(x['experiment_id'] == key for x in d['experiments'])
    d['experiments'].append(dict(experiment_id=key, baseline_commit='b19d6d7ce0cf40d4e55850ac856e9581734e1a6f',
        snapshot_ids=[ids[name]], hypothesis=hypothesis,
        changed_dimension='One EKF calculation epoch; preserved exact runtime inputs',
        result=result, reason=reason,
        production_impact='Participant EKF replaces underlay launch; no MPCC parameters or authority changes; full acceptance open',
        deleted_code=['underlay EKF launch/dependency', 'independent prediction/measurement/state-output clocks'] if suffix == 'repair' else [],
        revisit_condition=revisit))
p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + '\n')
print(len(d['snapshots']), 'snapshots;', len(d['experiments']), 'experiments')
