# Validation

## Static and unit validation

- Failure-first source contract failed before the implementation because the
  rate-resolved worker was submitted inside the synchronous five-state
  evaluator.  It passes after moving submission behind output resolution.
- `make autoware-build`: passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- CTest: 49/49 passed.
- Package test summary: 1808 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

## Dynamic validation

- Command: `make dev2`
- Artifact: `output/20260825-072127`
- Both d1 and d2 produced retained rate-resolved command candidates.
- Every candidate remained `authority=shadow, selected=0`.
- Fresh worker output was paired with its logged causal submission by source
  decision ID:
  - d1: 5 pairs, maximum displayed steering difference 0.00004876 rad.
  - d2: 37 pairs, maximum displayed steering difference 0.000049244 rad.
- The difference is below the 0.00005 rad rounding bound introduced by logging
  predecessor steering with four decimal places; no causal mismatch was seen.
- Control callback overruns above 25 ms: 0 on both domains.
- Typical callback averages were approximately 1.7--2.0 ms on d1 and
  4.3--4.9 ms on d2; observed maxima remained below 12.4 ms.
- The retained rate-resolved evaluator was approximately 0.205 ms average and
  0.322 ms maximum in the inspected summary window.
- Startup odometry-stale errors and shutdown process-termination messages were
  present.  No new runtime solver, non-finite command, or publisher error was
  observed from this Slice.

## Acceptance

The causal command-shadow Slice passes.  It proves that a six-state request is
born from steering which actually became committed control history, and that an
accepted retained proof can be exposed as a typed observation-only command
candidate.  It does not authorize that candidate to publish.

## Remaining blocker

Production promotion is still prohibited until the measured-to-control
connector and retained suffix have one explicit time origin.  Promotion must
then connect the six-state owner and remove the five-state Track/Cruise owner in
the same Slice.
