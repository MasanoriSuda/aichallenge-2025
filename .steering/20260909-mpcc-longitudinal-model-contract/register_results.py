"""Register diagnostic outcomes, including rejected and noncausal comparisons."""
from pathlib import Path
import json
import subprocess

path = Path('docs/spec/mpcc-experiment-registry.json')
data = json.loads(path.read_text())
prefix = '20260909-longitudinal-model-'
manifest = '.steering/20260909-mpcc-longitudinal-model-contract/evidence.json'
baseline = subprocess.check_output(['git', 'rev-parse', '1c4f377e^{commit}'], text=True).strip()
ids = {name: prefix+name for name in ['rest', 'single', 'moving-fit', 'physics']}
for key in ids.values():
    assert not any(row['snapshot_id'] == key for row in data['snapshots'])
    data['snapshots'].append(dict(snapshot_id=key, manifest=manifest, replay_ready=True))

rows = [
    ('flat-rest', 'rest', 'A flat observed moving loss may be carried over all future states and inputs', 'rejected',
     'Current native transition, explicit diagnostic flat -.37: rest wire0 yields -.074m/s after.2s. Motion-opposing isolated component preserves0. Real observer under stationary brake-3 yields residual+2.9998417301328315; extrapolating it to wire+1 gives+3.9998417301328315. No production repair.',
     'a causal input/state-dependent physical response model with independent rest tests'),
    ('single-observation', 'single', 'An instrumented single run can supply moving longitudinal response observations', 'accepted',
     'Six laps253.0538330078125s/penalty0/no moving override in monitor.11459receives/2630applications,10616exact wire matches.97318extracted ROS rows. Observation-only, not uninstrumented performance or new-model acceptance.',
     'changed binary/config/probe or missing observations'),
    ('flat-generalization', 'moving-fit', 'One moving training residual generalizes across single and dev2', 'rejected',
     'Single15..90s training flat=-1.2782374350670689m/s2 improves single holdout but independent pre-Emergency dev2D2 increment MAE worsens .01834937804855 to .02638893119782m/s. Separate native rest rejection remains.',
     'new causal conditional response model, never the same fixed offset retuned to the failure'),
    ('euler-raw', 'moving-fit', 'Raw VelocityReport.heading_rate directly represents an unwrapped body yaw derivative', 'rejected',
     'Decoded producer directly subtracts absolute Euler angles. Single has5winding samples. Analysisr2 recovers each principal increment before midpoint; r1 kept. Euler/body and float-quantization limitations remain. Controller uses Odometry, not established933cause.',
     'changed sensor producer or independently synchronized body-angular evidence'),
    ('wheel-conditional-fit', 'moving-fit', 'Measured-state-conditioned wheel force terms reduce moving increment error', 'accepted',
     'Train15..90s2143intervals;holdout90..250s4571. MAE wire->wheel .0464864567->.0105816938m/s; independent pre-Emergency dev2D1 .0241227135->.0071666354,D2 .0183493780->.0017682871. No refit; raw Euler winding handled explicitly.',
     'new independent observations or causal model evaluation'),
    ('wheel-promotion', 'moving-fit', 'The measured-state-conditioned fit is ready for canonical production MPC', 'inconclusive',
     'Fit uses future measured midpoint v,vy,yaw,steering and diagnostic actual application, neither available as future controller input. Low-speed/rest excluded. Contact/force epoch and independently propagated lateral states unresolved; no parameter promotion.',
     'causal held-out prediction, input/contact uncertainty contract and complete physical proof'),
    ('runtime-physics', 'physics', 'Effective local vehicle values and contact can be read with bounded diagnostic probes', 'accepted',
     'Two receivers each1body+4wheels captured then teardown, no probe errors. mass160;front sprungMass~25.881kg, onefrontwheel ungrounded each. YAML grip .6 clamped runtime .7; explicit Player.log warnings. Same3608method-normalized main DLL and4calls as prior probe; new read-only helper, no timing parity claim.',
     'changed local simulator/config or synchronized full force observation'),
    ('uniform-contact', 'physics', 'Equal and continuously grounded four-wheel force is a valid observed plant assumption', 'rejected',
     'At both moving observations onefrontwheel reports ungrounded; front total sprungMass fraction~.16176, groundedfront~.08985. Decoded Wheel.UpdateWheelForce returns without lateral/drive force if ungrounded. Only two samples, not full-time contact history or force measurement.',
     'different proven contact regime or a model covering contact uncertainty'),
]
for suffix, name, hypothesis, result, reason, revisit in rows:
    key = prefix+suffix
    assert not any(row['experiment_id'] == key for row in data['experiments'])
    data['experiments'].append(dict(experiment_id=key, baseline_commit=baseline,
        snapshot_ids=[ids[name]], hypothesis=hypothesis,
        changed_dimension='Observation-only native/rest and moving conditional model comparison; one-time generated helper physics probe',
        result=result, reason=reason,
        production_impact='No controller/model/gain/delay/solver/limit/authority changes. Full MPCC acceptance open; concrete completion plan updated.',
        deleted_code=[], revisit_condition=revisit))
path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
print(len(data['snapshots']), 'snapshots;', len(data['experiments']), 'experiments')
