# Received pose-bracket diagnostic, frozen before force-r3

2026-09-13 JST. Production observation commitfe976b39/build121/package108.
This is a measurement protocol, not a production observer or parameter change.
Rollback remainsfe976b39. M4–M6 completion remains open.

R563 proves real controller receipt of body samples excluded by the pose cutoff.
R7 has20unique decision captures (26valid serialized occurrences), including
source859 with a20ms-old velocity and an already received upper sample. R564
measures35ms velocity/tire and50ms IMU source periods. R7 has no private truth at
its pose epochs, so it cannot establish interpolation accuracy. R566 rules out
changing the command-derived initial desired steering as a nominal prediction
cause in its7recorded comparisons; history supplies the applied desired input.

Freeze `received_pose_bracket.py` before a new force-r3 observation. Compare:

- Original numerical body/tire initial state: raw components held at pose epoch.
- Per-component linear interpolation at that same pose epoch, only when both
  endpoints are in the exact immutable controller capture and no endpoint is
  newer than the capture's own decision `now_sec`. If no upper endpoint exists,
  retain the original held value and label that channel unresolved.

Preserve raw endpoints, their original source and steady receipt timestamps,
interpolation fraction, effective pose epoch and original/derived values. Desired
steering is command-derived and stays the original observation's value. Position
and orientation are unchanged. No extrapolation, thresholds, clipping, fitting,
new runtime input, loosened physical/clock gate or claim of an error bound.
R565 exercises finite bracketing, missing upper support, clock-newer exclusion,
receipt/order/owner failures and all R7 captures. This is implementation evidence,
not a physical score. Never relabel an interpolated value as a raw sample.

Use the existing validated seven-call force probe and unchanged build121. Run
at most120host seconds, with the existing first moving final override/actual
publication violation stop and6lap/600s simulator configuration. Do not modify
controller source/config during the run. Capture source/binary/probe hashes,
restore the original DLL and protected JSON, and stop all containers afterward.
The probe changes timing; this is not race acceptance.

Primary cohort: every unique valid received capture available in the new run,
with aliases preserved and deduplication by control decision ID plus captured
steady timestamp and actual selected/history payload. Keep pre-Ready/Ready,
stationary/moving and failed/accepted-source provenance separate. Do not select
captures by speed or score. Compare both arms' u/vy/body-r/physical-tire values
against the same physical sample at that pose epoch using the shared explicit
base_link/COM reader. Require same Domain/vehicle and timestamp alignment; report
unmatched samples separately instead of inventing physical truth. Preserve all
raw source files and hashes. Report per-channel sample count, mean/RMS/maximum
absolute error, signed errors and both improvements/regressions. Report exact
pose-stamped, interpolated and unsupported held channels separately; rest/launch
and stopping cases must not be hidden in pooled means.

This primary comparison closes only the observation reconstruction boundary.
Any later prediction comparison is supplementary: state initialization and
contact/actuation errors remain distinct, with actual future inputs explicitly
labelled oracle inputs. Production adoption requires explicit derived-state/raw
provenance, coherent same-model proof/identity, original negative regressions,
clock/reset/rest checks, and closed-loop acceptance. An offline improvement alone
does not authorize publication or complete remaining M4–M6.
