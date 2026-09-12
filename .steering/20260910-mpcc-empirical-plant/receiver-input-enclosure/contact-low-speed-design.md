# Contact forecast comparison and prospective low-speed observation

2026-09-13 JST. Controller8d9810e0/build120/package107 unchanged. This is diagnostic
model comparison within the accepted empirical-simulator scope, not an additional
normal authority or a production parameter update. FullM4–M6 remain incomplete.

R545/R546 compare fixed transition laws (joint16states/independent4wheels, raw or
mirrored counts) trained on old single16–35s. Initial last/past20contact is private.
All1753baseline scores exactly matchR544. Improvements are small; no deployable
observation or complete source/current/Stop proof follows.

R547 exposes a calibration limit: all3800old training samples are at5.232–8.342m/s
with almost constant wire1.329596m/s² (standard deviation3.35e-7). The longitudinal
design condition is2.984e7. Full numerical rank and optimizer convergence do not
identify a response to braking. Its wire-conditioned fits are not accepted as
such a response. R548 instead fits speed/lateral contact probabilities on the same
training only. New1sspeed MAE improves0.138748→0.074906m/s; oldholdout position
0.170395→0.153570m. OldD1position/yaw regress, including6half-second anchors; its
1smean has only one anchor. A global production replacement is not accepted.

R549 explicitly adds r1awakeDrive8–10s,400samples at0.183–1.153m/s and wire−3 to
1.3296m/s². Old high-speed training stays16–35s. No old holdout/dev2 or force-r2
training. Mirror-tied per-axle binomial contact laws use own predicted u²/g,
|u*r|/g and side*u*r/g, with/without wire/g. Physical force coefficients, margins,
timing, normal constraints and rest law remain unchanged. Design conditions are
15.71/122.28. This calibrates nominal contact probability, not a physical bound.

All500R539anchors retain correct base_link truth; the three exact snapshots retain
their public mixed-epoch initial values. The r1training-overlap and later known
regression groups are separate. R1was already inspected, so its later segment is
not untouched validation. The exact programme145msspeed error falls0.027035→
0.002049/0.003193m/s; its yaw error and oldD1regressions remain. No model is promoted. R549report's last generic limit line still says old training
only; that inherited wording is stale. Its frozen-model coverage and explicit
scope correctly record old3800 plus r1 8–10s400samples. No numeric result or
coefficient is changed by this metadata correction.

## Frozen prospective observation

Before force-single-r2, seal R548/R549 coefficient/model/driver hashes in
[evidence](contact-forecast-evidence.json). The runner verifies and records those
hashes before launching the unchanged current controller with the validated
seven-call force/application DLL. Original6laps/600s configuration,120s host cap,
first moving Emergency termination, complete teardown and protected restoration
remain. This diagnostic observes a new world; it is not an unchanged race rerun
offered as a repair, and the instrumentation changes timing.

Afterward extract the same physical/public/application streams and exact first
failure. Compare the frozen models at fixed20-physics-step anchors, only before
the first failure, horizons0.1/0.25/0.5/1s when available. Keep actual future wire/
tire as the declared exogenous boundary in every arm, and source/receipt/physics
times separate. No fit, window search or coefficient change on r2. Report missing
coverage and each per-world score; do not pool away oldD1or exact-source yaw errors.

Promotion would still require a causal public observation/model definition,
coherent nine-state QP/native/Stop/current/async identities, original hard physical
and scheduling gates, negative regressions, full package/build tests and fresh
uninstrumented acceptance. Private-contact initialization or future-contact oracle
cannot execute. A rejected comparison stays diagnostic; no unused production
model or observer is installed. Further modelling must explain residual contact
regime differences, rather than accept an unsupported fit or weaken a gate.
