"""Preserve handoff repair acceptance and unresolved coupled results."""
from pathlib import Path
import json
import subprocess

p = Path('docs/spec/mpcc-experiment-registry.json')
d = json.loads(p.read_text())
prefix = '20260909-publication-clock-'
manifest = '.steering/20260909-mpcc-publication-clock-handoff/evidence.json'
baseline = subprocess.check_output(['git', 'rev-parse', '17bcf24b^{commit}'], text=True).strip()
ids = {name: prefix + name for name in ['native', 'world937', 'single-r1', 'dev2-r1']}
for key in ids.values():
    assert not any(x['snapshot_id'] == key for x in d['snapshots'])
    d['snapshots'].append(dict(snapshot_id=key, manifest=manifest, replay_ready=True))
rows = [
    ('native-before', 'native', 'Same-artifact Bundle-to-exact handoff preserves the published clock', 'rejected',
     'Actual Store: 32 cases pass; new boundary fails with 0.0044038810139195306 s cursor rewind, matching captured 936/937.', 'new Store implementation or publication semantics'),
    ('repair', 'native', 'Transfer the published proof/command anchor on same-artifact handoff', 'accepted',
     'All 33 Store cases pass; 26-package build; MPCC 60 CTest groups/2373 records, 0 errors/failures/skips. Ordinary continuous clock and stale decision rejection preserved.', 'new publication clock regression'),
    ('anchor-only', 'world937', 'Correcting the anchor alone rescues old exact 937', 'rejected',
     'Zero-solve cursor 0.4094038720139199 s; terminal still rejects at -0.0014001134308958552 m. All measured state and peers unchanged.', 'demonstrated additional causal producer change'),
    ('normal', 'world937', 'A/B/C/D/G normal candidates rescue sealed 937', 'rejected',
     'All nine bounded arms solver-reject with unchanged model/hard limits. No physical infeasibility certificate; Unknown.', 'new bounded formulation or causal input defect'),
    ('support-stop', 'world937', 'Existing support Stop methods rescue sealed 937', 'rejected',
     'All four solver-reject; no physical infeasibility certificate.', 'new bounded witness or causal input defect'),
    ('single', 'single-r1', 'Fresh single-car six-lap acceptance', 'accepted',
     '251.993591 s, penalty 0, no observed moving override, max callback 13.748 ms/0 overruns; command/Odometry/clock receipt and source checks have no >50 ms gaps or source duplicate/backward.', 'relevant production change or new regression'),
    ('runtime-clock', 'dev2-r1', 'The repaired handoff preserves mapping at new 928/929 boundary', 'accepted',
     'Same-376 anchors are equivalent: cursor advances 29.999999 ms with the observation. Anchor-only substitution preserves exact 929 result. Throttled observation is not all-cycle evidence.', 'new inconsistent paired boundary'),
    ('race', 'dev2-r1', 'Published-clock repair establishes coupled race acceptance', 'rejected',
     'First moving D1 Emergency 930 at 1.3035718076716887 m/s; no finish. D1/D2 max callback 28.673/27.375 ms and 1/3 overruns are in reported windows after first Emergency.', 'causal production change or missing diagnostic evidence'),
    ('peer-switch', 'dev2-r1', 'Peer update can flip exact 928/929 terminal acceptance', 'accepted',
     'Zero-solve originals: 928 +0.01128045374819564 m; 929 -0.0007831556743211898 m. Later peer field makes 928 reject -0.00023400897409575627 m; earlier peer field makes 929 accept +0.011806859326646713 m. Counterfactual only, no producer-fault or authority claim.', 'audit actual peer source/receipt/velocity clocks'),
    ('derived385', 'dev2-r1', 'Final 930 has complete derived Stop 385 artifact/proof evidence', 'inconclusive',
     '929 independent Stop accepts +0.002328087195587747 m and derived 385 joins. Final Request present but actual-publication and inspected artifact serialization require missing solver source. Prior Accepted 928/376 invalid for 385.', 'preserve actual derived artifact and proof/derivation provenance without a replacement solve'),
    ('delivery', 'dev2-r1', 'Source and receipt streams are continuous', 'rejected',
     'Command receipt about 40 Hz; D1 one gap 50.877094 ms, D2 maximum 35.742760 ms. Source duplicates/backward 3 each, gaps 2/1; Odometry maxima 95.826387/100.583076 ms, clock 96.961260/98.612309 ms.', 'causal source/receipt investigation or relevant producer repair'),
]
for suffix, name, hypothesis, result, reason, revisit in rows:
    key = prefix + suffix
    assert not any(x['experiment_id'] == key for x in d['experiments'])
    d['experiments'].append(dict(experiment_id=key, baseline_commit=baseline,
        snapshot_ids=[ids[name]], hypothesis=hypothesis,
        changed_dimension='Actual published source clock handoff; named sealed-world comparisons',
        result=result, reason=reason,
        production_impact='Store clock transfer only; no new authority/model/limits; full acceptance open',
        deleted_code=['same-artifact old execution anchor retained across newer Bundle publication'] if suffix == 'repair' else [],
        revisit_condition=revisit))
p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + '\n')
print(len(d['snapshots']), 'snapshots;', len(d['experiments']), 'experiments')
