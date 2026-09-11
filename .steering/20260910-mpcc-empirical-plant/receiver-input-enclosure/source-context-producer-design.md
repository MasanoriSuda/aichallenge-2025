# Source generation propagation and invalidation

Baseline d94aa556. C5 integration is ongoing. The old synchronous normal path
still owns publication. Its retirement is mandatory in the new dispatch promotion
slice. There is no new normal authority in this source-provenance slice.

## Actual producer binding

The context primitives live in mpcc_scheduled_context.hpp so the actual numerical
Snapshot can own normal_context_generation without an include cycle. The live MPC
captures it in RateResolvedTrackCruiseSubmissionDraft, before solve, and carries it
through BoundRateResolvedTrackCruiseSubmission into the solver Snapshot. Existing
pipeline/CertifiedPlan/Store retain that same solver_source_snapshot. Real stateless
left/right and time-aligned feedback/suffix tests check preservation and revocation.
Tactical MPC copies borrow the generation; their simulated local mutations cannot
revoke the parent. Numerical fingerprints and serialized snapshots exclude this
runtime revocation capability. Loaded historical diagnostics have no live authority.

The scheduled factory now reads context ONLY from the plan's solver_source_snapshot.
Request.source_context was removed. Callers cannot stamp a present generation onto
an old plan. Source-less diagnostics can still compute a numerical certificate,
but the existing current-context gate rejects it. A test completes a proof after
its source was revoked and checks that a newly captured generation cannot revive it.

## Mutable producer coverage

- Existing true async/solver/control-history resets, changed v_max/ay_max, Q/R/QN,
  filter gains and waypoint preview values revoke before mutation. Unchanged
  effective v_max setters at40Hz preserve context. Parameter callbacks may partly
  mutate before a later error, so invalidation occurs before each changed value.
- Live ReferencePath binds the MPC invalidator. Changed speed values, nominal
  profile, simple/external path bounds and border cells revoke before assignment.
  Invalid messages or identical values preserve the generation. New bound/cell
  arrays are prepared before changing live arrays. Restoring values does not
  restore a revoked source. Unused mutable-waypoint access was removed.
- ReferencePath has an explicit value-copy constructor that omits the live
  invalidator; tactical copies cannot revoke the parent. Reference replacement
  revokes before swapping, including a failed reassociation that restores values.
  Width/length/reset mutation entry points also invalidate before writes.
- Changed race session, control enable/stop request, gear changes, Recovery reset
  and actual Recovery command arbitration, invalidated frozen mission, wall-grid/
  planner replacement, decision sequence rollover, observed clock regression,
  ledger-record failure and shutdown revoke their live source generations.
- Atomic intended mission/phase adoption is a separate remaining audit: blindly
  revoking at mission commit would destroy the prospective incoming source. Current
  semantic and reference compatibility must be checked before actual admission.

## Evidence and limits

R269 is a diagnostic compile setup failure (old node had no scheduled_control alias),
not a controller regression. R270 fixes the diagnostic alias and compiles the real
old ReferencePath/Map methods. Eleven of29checks fail because source revocation is
absent. R271 compiles the changed methods:30/30pass, including a callback observing
that invalidation precedes the first changed write, unchanged values, rejected
inputs, profile/bounds/cells, restoring values and independent worker copies.
The extra check runs only when the changed implementation calls the invalidator.
The baseline compatibility adapter makes absent invalidator registration a no-op;
it does not modify old setter behavior. This is a focused native test, not ROS
callback or race coverage.

Buildr77/testsr66:26packages/2509records; r78/r67:26packages/2510records;
final r79/r68:26packages/2510records/64groups,0errors/failures/skips.
All affected libraries/tests were rebuilt after Snapshot layout changed. Historical
native/replay binaries must be recompiled before using the current Snapshot and
scheduled Request definitions. No new simulation is claimed: dev2-r44 still fails
actual publication timing and has no Follow coverage.

## Remaining C5 and acceptance

- Complete mission geometry/prospective atomic transition compatibility and audit
  all derived candidate producers. Missing or revoked source context rejects.
- Capture a real post-publication observation/history/last-issued angle and its
  dependent point prefix, peer epoch and Follow branch. Changing only now is invalid.
- Integrate one proof worker and one current-evidence/final-clock/slew dispatcher.
  Retire replaced synchronous normal/Stop/GateA/previous-intent joins in that same
  promotion slice. Preserve source-job versus current-dispatch decision identity.
- Keep exact integer epochs,25msraw windows, original steering serialization,
  empirical250ms profile,130mscontrol origin and100mssteering delay unchanged.
- Run actual timing/intent/Stop/rest/restart, then finalHEAD M4–M6and submitted eval.

[Evidence](source-context-producer-evidence.json),
[adoption gate](scheduled-adoption-boundary-design.md),
[current evidence](scheduled-current-evidence-design.md),
[dispatch obligations](scheduled-context-dispatch-design.md).
