"""Register all supported, rejected and unresolved shared-model comparisons once."""
from pathlib import Path
import json
import subprocess

path = Path('docs/spec/mpcc-experiment-registry.json')
data = json.loads(path.read_text())
prefix = '20260910-shared-plant-'
manifest = '.steering/20260910-mpcc-shared-plant-model/evidence.json'
assert Path(manifest).is_file()
baseline = subprocess.check_output(['git', 'rev-parse', '9bca3af6^{commit}'], text=True).strip()
names = ['base-link', 'public-inputs', 'contact-estimate', 'body-identification',
         'steering-source', 'native', 'contact-epoch', 'cached-contact', 'stop-boundary', 'wheel-colliders']
snapshots = {name: prefix+name for name in names}
for key in snapshots.values():
    assert not any(row['snapshot_id'] == key for row in data['snapshots']), key
    data['snapshots'].append(dict(snapshot_id=key, manifest=manifest, replay_ready=True))
rows = [
    ('root-origin-assumption', 'base-link', 'Rigidbody origin may be used as controller base_link without a lever-arm transform', 'rejected',
     'Serialized base_link is0.485m rearward and0.05m upward; COM is+0.175992m forward of base_link. Earlier root-origin body scores retained but cannot establish the controller lateral-state requirement.',
     'changed serialized/observed pose origin or an explicit correct coordinate transform'),
    ('absolute-gnss-position', 'base-link', 'Measured-heading GNSS output has the same horizontal base_link origin as independently transformed physics', 'accepted',
     '51exact-source comparisons perDomain, fixed serialized MGRS offset, no fitted translation. Maximum XY component4.012291mm. Vertical/geodetic conversion explicitly excluded.',
     'changed GNSS/IMU producer, calibration, map datum or vehicle origin'),
    ('causal-dynamic-comparison', 'public-inputs', 'Dynamic lateral/yaw propagation improves the declared public-input body prediction comparison', 'accepted',
     'Corrected base_link single0.5s position/speedMAE legacy.194635/.628518, reduced force.095839/.116810, dynamic.056545/.066474. Fixed16..35scontact/physical averages; future prescribed publications only, future sensor/contact scoring only. Dynamic1smaximum.675902m remains; D2short.1s position does not improve.',
     'new model structure, independently frozen evaluation or new source-epoch evidence'),
    ('past-contact-promotion', 'contact-estimate', 'Past public sensor least-squares contact weights improve general prediction sufficiently for promotion', 'rejected',
     'Past.5ssensors and stable published wire;490single estimates and2missing early anchors perdev2Domain. Root-origin single.5sMAE.066253->.080008m and1smaximum.739249->1.449915m worsen; D1worse,D2better. No coefficients promoted.',
     'new causal contact-state information; do not retune the same observer window against this failure'),
    ('four-coefficient-promotion', 'body-identification', 'Four fitted axle drive/corner weights provide a generally improved model', 'rejected',
     'Only16..35straining,93anchors,8optimizer evaluations; training RMS.033603->.032273m equivalent. Root-origin held-out1smaximum worsens to.774864m and bothdev2speed errors worsen.',
     'new causal model structure or independent contact/terrain measurements'),
    ('steering-source-only', 'steering-source', 'Source-propagating the last tire observation resolves the public-input discrepancy', 'rejected',
     'Past publication history and inspected.1smechanical delay; root-origin single.5spositionMAE.066253->.067273m, little dev2change. Actual DDS/receive timing remains unknown; no delay fit.',
     'new source/application evidence or a distinct actuator-state contract'),
    ('actuator-drive-parity', 'native', 'The inspected post-receiver steering and Drive/contact sleep law can be reproduced natively', 'accepted',
     'Strict C++17native checks and20147physics ticks/80588wheel values pass. Maximum tire1.90735e-6degree, wheel2.88142e-7m/s2, sleep0mismatches/1641D1sleep ticks. All recordedgear=3. First.3sunknown tire history excluded. Private receiver/contact are exogenous diagnostics.',
     'changed simulator component/config, non-Drive requirement or new recorded component mismatch'),
    ('previous-contact-frame', 'contact-epoch', 'Cached WheelHit directions correspond more closely to previous body/tire than current state', 'accepted',
     'Separate0/1/2tick body/tire combinations. Single front directionMAE.003670/.004076->.00004180/.00004301rad forpreviousboth; rear likewise and bothdev2Domains improve. Maximum residual.003388rad remains; not exact parity or DDSdelay identification.',
     'changed execution order/physics or independently synchronized contact evidence'),
    ('cached-frame-promotion', 'cached-contact', 'Correcting the cached contact epoch alone provides a promotable body model', 'rejected',
     'Causal previous predicted body/tire and points, no future contact, same frozen coefficients/public initial data. Single.5spositionMAE.056545->.055794m but1sMAE.193661->.222850m,maximum.675902->.692849m worsen. Component finding is not sufficient model repair.',
     'new contact/terrain/force-state information or a structurally different model'),
    ('serialized-friction', 'wheel-colliders', 'Serialized WheelColliders supply additional hidden tangential friction', 'rejected',
     'Read-only local level1 extraction: all16forward/sideways friction curves/stiffness arezero. Suspension spring35000/damper3500/distance.001m remains active; no claim all normal/contact forces are absent.',
     'changed serialized/runtime WheelCollider values or actual tangential force evidence'),
    ('publication-application-equivalence', 'stop-boundary', 'One public25msbrake stream defines one applied physical trajectory under10Hzlatest-value selection', 'rejected',
     'Native conditional straight/all-ground component: selectionphase0vs30ms yields0.2sspeeds1.255407vs1.690412m/s fromthe same received stream. Immediatevs.5sdelayed continuous brake changes modeled stopping support. Simplified distances are not actualAWSIM measurements; unconstrained no-contact branch is not road infeasibility proof.',
     'explicit bounded application semantics or feedback contract, never a fitted single publication shift'),
    ('model-promotion', 'base-link', 'The full shared plant/Stop model is ready to replace canonical production authority', 'inconclusive',
     'No candidate yet closes contact/application uncertainty or complete Stop. Currentdev2rejected; user requirement question distinguishes empirical simulator acceptance from guaranteed bounds. No reply/observed maximum is treated as authorization or guarantee. M1-M6 remain open.',
     'resolved acceptance requirement, selected causal model with explicit uncertainty and independent Stop/trajectory validation'),
]
for suffix, name, hypothesis, result, reason, revisit in rows:
    key = prefix+suffix
    assert not any(row['experiment_id'] == key for row in data['experiments']), key
    data['experiments'].append(dict(experiment_id=key, baseline_commit=baseline,
        snapshot_ids=[snapshots[name]], hypothesis=hypothesis,
        changed_dimension='Diagnostic public-input model, corrected pose/contact epochs and conditional actuator/Stop contracts',
        result=result, reason=reason,
        production_impact='No controller, model, solver, gain, delay, limit or authority change. Coupled/full MPCC acceptance remains open.',
        deleted_code=[], revisit_condition=revisit))
path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
print(len(data['snapshots']), 'snapshots;', len(data['experiments']), 'experiments')
