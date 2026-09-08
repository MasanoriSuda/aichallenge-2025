# Audit order

1. Preserve map/binary/MCAP/results and classify the first prefix rejection.
2. Inspect frozen solver and actual nonlinear physical comparisons. Overtake
   architectures with no target are inapplicable, not successful alternatives.
3. Join command, steering report, body motion and sensor timestamps. Inspect
   the local AWSIM input/actuator/steering-report implementation and units.
4. Separate model/calibration error, state estimate error, timing error,
   conservative projected geometry and numerical approximation. A larger
   margin, shorterdelay or suppressed rejection is not a causal repair.
5. Establish a failing native/consumer reproduction before production change.

The independent peer-envelope audit remains recorded and is not promoted
into the current single-vehicle failure without evidence of relevance.

## Bounded observation slice

At the first normal-authority-loss boundary, retain the existing atomic
latest-published Store snapshot before Emergency clears it. The current-world
snapshot and original source/artifact record run in ONE existing asynchronous
worker job. The writer retains per-intent/homotopy/outcome deduplication.
No new Store, clock, solver, authority or retry rule is introduced.

The source record includes the actual artifact states/controls/model/certificate
fields plus its original complete solver input. It records the publication
ledger kind, control-origin/cursor, latest publication decision and a fingerprint
back-reference to the current failure. For exact execution the clock is the
FIRST publication clock; latest decision remains a separate field. A current-
world Bundle source is labelled as source evidence, not exact execution.
Artifact/source identity mismatch is rejected rather than silently combined.

Promotion boundary: NONE. Offline evidence cannot command the vehicle. Retain
the recorder as diagnosis only while this failure is unresolved; remove the
controller capture if callback/storage overhead cannot stay within the existing
budget. Regression verifies actual values, independent clocks and wrong-source
rejection, followed by package/build and one bounded diagnostic.
Rollback baseline: a8b968ac2e4a86a28432a52d132774459d355cb5; observation slice is
limited to architecture_snapshot hpp/cpp, its test/CMake linkage, and the existing
controller observation helper. Preserve all earlier unrelated working changes.

## Live detection gap corrected after diagnostic2

Diagnostic2lost normal authority at9005(terminal-contingency-unavailable,
not delay-prefix collision). The current snapshot was written but original
source was absent: evaluate_rate_resolved_pipeline retained solver input ONLY
for ShiftOut/Pass. The initial recorder therefore could not observe Cruise.
An actual-producer control-flow probe compiles the exact capture prolog and
drives all7normal intents: before5missing/2owned; after7owned, with original
mutation proving independent immutable ownership. No copied alternate policy.

Remove the intent filter from worker-side source capture. Keep existing bounded
Store/worker lifetimes and shared wall grid, and permit no new authority use.
All source-pointer consumers were audited: plan identity validation, pending
publication provenance, ShiftOut/Pass-gated observation and this failure
recorder. Normal behavior/solver/physical guards and parameters are unchanged.
The first successfully recorded current failure now logs a missing source
explicitly rather than silently omitting it. Rebuild/package and final bounded
diagnostic follow. Controller v1binary/results remain sealed separately.

## Completed observation and physical-model comparison

V2build/tests pass; r3captures actual published source6782and failure7405.
No more unchanged diagnostic runs. Native artifact reconstruction uses zero
solver calls. [Frame-model audit](frame-model-audit.md) establishes that the
current virtual-progress Frenet transition and world reconstruction are
inconsistent, and the same saved controls change native wall reserve verdict
under independent Cartesian integration. Correct rotating-frame terms alone
still leave a separate course-interpolation mismatch. Compare complete
coordinate-consistent formulations before a production model replacement.
The observation slice is complete; the controller repair remains open.
