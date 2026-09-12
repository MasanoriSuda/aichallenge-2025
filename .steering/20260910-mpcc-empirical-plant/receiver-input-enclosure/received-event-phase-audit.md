# Received event reconstruction and vehicle-update phase

2026-09-13 JST. Baseline54964eb4, controllerfe976b39/build121/package108;
model8d9810e0 unchanged. R570–R575 are offline diagnostics. Coupled observer
**not promoted**; fullM4–M6 incomplete. [Evidence and payload hashes](received-event-phase-evidence.json).

R570 supports19of25captures with directly matched command history. R571 restores
current883/1005using their own original source ledger cursor and contiguous actual
transactions through last publication. It preserves the original observation and
separately records the join.21native reconstructions succeed, with exact equality
of body states under an arbitrary global translation/yaw; four early inputs lack
common epoch/history. All16primary physical matches remain. No later-capture
history, private seed, future-to-pose measurement or zero fallback is introduced.

## Correct physical comparison phase

ForceProbe.Begin records Rigidbody body velocity before Vehicle.FixedUpdate.
End records actualSteerAngle after that update. Public steering at the same source
timestamp matches the immediately preceding contiguous End tire, hence the tire
before the current update.21unique tire endpoint errors have maximum1.0533e-8rad
against Before, versus0.00698132rad against End.42u/vy endpoint components match
Before to1.708e-7m/s; IMU body-r is a finite quaternion difference and retains its
own-source error (maximum0.000729568rad/s).

The shared reader's legacy default remains bit-exact. A separate explicitly named
body-phase function exposes direct Begin body and inferred Before tire; first
rows/gaps/identity changes are unsupported, with NaN tire rather than zero.
Five negative checks reject ambiguous phase or invalid continuity. No sensor
stamp, control threshold, pose origin or native dynamics is changed.

R572 preserves every R568/R569/R571 estimate and the16matched/9unmatched primary
cohort. Begin/End body values are equal at these16matches. Correct mean errors:

| Component | Held | Frozen bracket | Coupled event |
|---|---:|---:|---:|
|u,m/s|0.01177593|0.00589199|0.00572207|
|vy,m/s|0.00134563|0.00089558|0.00171589|
|r,rad/s|0.00351325|0.00188984|0.00373127|
|tire,rad|0.00147651|0.00141576|0.00351833|

Frozen interpolation also worsens maximum tire error from0.00548674to0.01182532rad.
Coupled maximum tire error is0.02759241rad. Neither blanket reconstruction is a
validated production repair. Corrected R540 has **zero** before-tire tube misses
across30samples (old End-phase count12); its24speed misses remain. R557 retains
8tire misses/27samples and its original body failures. These corrections do not
loosen a bound or imply complete body containment.

R575 corrects another phase distinction: integrating the force at End(j) predicts
Begin(j+1), before the next Vehicle Sleep override. Exactly5of11140R555targets
change, all next-Sleep transitions. Native force predictions and derivative errors
remain unchanged. Current3Dforce still improves lateral/r errors relative to the
planar contact oracle; velocity/contact/constraint and rest issues remain.

## Isolate receiver timing from actuator and body dynamics

R573/R574 compare the same15supported public seeds and source events on the actual
physics grid. Decision787has no recorded physics at its common seed and remains
unavailable; it is not silently dropped from the original16case primary cohort.
R573 incorrectly fed physical tire demand into native desired steering without
dividing by tire_grip*wire_gain. Its two private arms are superseded, retained
unaltered. R574 corrects the conversion and exactly preserves nominal-grid output.

R574's actual applied wire plus observed Unity float-queue delayed demand yields
maximum tire error2.3852e-8rad; imposing actual tire yields1.0533e-8rad. Thus the
existing mechanical response explains these records once the actual input/time
is supplied. The public-history nominal-grid maximum is0.02072385rad. Nominal
fixed input timing cannot be conflated with observed receiver/application timing.
Forward-speed MAE falls0.00562993→0.00239903m/s with actual inputs, while lateral
MAE remains0.00172490m/s, worse than original held0.00143338m/s on the same15.
Actual tire does not repair body/contact dynamics; native rest also remains an
approximation near physical recoil (1005). These private arms cannot initialize
or drive a production observer.

Next isolate the missing current force terms (3D body rotation/velocity, contact
geometry and solver impulses) against the retained force balance before a new
model fit. Preserve separate input timing, source-epoch reconstruction and body
model hypotheses. Any production change must fix its producer and reseal all
nine-state/native/QP/Stop/current/async consumers, preserve the original hard
negative replays, then pass build/package and new dynamic acceptance. No repeated
unchanged live run or parameter/margin workaround is justified by these results.
