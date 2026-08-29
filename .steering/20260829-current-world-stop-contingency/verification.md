# Verification

## Static verification

- `make autoware-build`: passed, 25 packages.
- `test_mpcc_rate_resolved_physical_adapter`: passed.
- `test_mpcc_execution_contract`: 75/75 passed.
- `test_mpcc_rate_resolved_retained_revalidation`: 48/48 passed.
- `test_mpcc_rate_resolved_command_candidate`: 12/12 passed across
  command-candidate and production-adapter suites.

## Dynamic verification

Run: `output/20260829-115006` (`make dev2`).

The frozen `output/20260829-111010` failure stopped D1 around decision 1034
because a future dynamic intersection reduced the normal proof to one stage and
no terminal contingency existed. In this run D1 reached the equivalent region
at decision 1036 and logged:

- normal dynamic scope: `current-stage-prefix`;
- terminal Stop: `attempted:1/certified:1`;
- terminal wall proof: valid and clear;
- terminal dynamic proof: clear, minimum clearance `1.113 m`;
- final retained decision: accepted.

Both domains subsequently exercised certified Stop suffixes. No wall-entry,
actual-footprint-wall-margin or Overtake Recovery event was logged before the
run was stopped. Callback telemetry remained below the 25 ms period with zero
overruns.

## Newly exposed defect

D1 later failed at decision 1330 through a separate chain:

`progress-lift-rejected -> cursor-unavailable -> no fresh/retained canonical
authority -> Emergency Stop -> stuck recovery`.

This is not repaired in this Slice. The Stop suffix did its scoped job and
exposed a progress/cursor lifecycle defect farther downstream. The next Slice
must audit the progress lift, artifact clock and fresh-solution supply as one
causal chain rather than add another grace period or fallback.

## Remaining model risk

If stopping distance extends beyond the artifact stages, the Stop rollout
currently holds the last certified stage curvature and bounds while world-pose
reconstruction still has to succeed against the immutable course-frame knots.
The final swept-wall proof fails closed if reconstruction is unavailable, but
this extrapolation remains a model/certificate mismatch risk to revisit if it
appears in dynamic evidence.
