# Findings

## Observed phenomenon

Baseline `a8b8cbe3`, run `output/20260831-060156`, D2 decision 1483 entered
ShiftOut and then lost normal authority. The first numerical failure was the
sequence-864 dynamic-obstacle refinement QP, not the later target-stale
Recovery.

## Causal chain

The current control epoch sealed one stage-wise target tube into the canonical
seven-state problem. Stateless candidate generation discarded it and ran a
second target predictor against the finite ego wall-course window. Projection
reached the final course knot, pinned target progress there, and interpreted
the remaining global displacement as lateral motion. At stage 10 it required
both `e_y >= -3.836 m` from the wall and `e_y <= -6.236 m` from the selected-side
obstacle constraint. OSQP could not satisfy the contradictory rows. Emergency
Stop and `locked target stale or lost` were downstream effects.

## Root cause

Target prediction had two owners inside one immutable control epoch. The
candidate rebuild was allowed to replace the upstream current-epoch target
tube with a coordinate-window-dependent approximation.

## Existing patch interaction

The failure snapshot is captured after candidate refinement and already stores
the old rebuilt tube plus `forced_physical_diagonal=true`. It is replay-ready
for the old contradictory QP, but it cannot reconstruct the raw pre-candidate
tube and therefore cannot validate the ownership repair by itself.

## Implemented change

- Replaced `rebuild_target_horizon` with
  `resolve_canonical_target_horizon`.
- Stateless candidates now validate and copy the sealed current-epoch target
  stages without modifying their geometry.
- Missing stages, target-ID mismatch, observation-generation mismatch,
  non-finite geometry, or invalid physical extents fail closed.
- Removed the duplicate course projection and its endpoint-clamp taxonomy.
- Kept ReplayWorld as the independent input to exact timed physical proof.

No Mission rule, lease, grace, timeout, retry, fallback, solver setting,
clearance, weight, or configuration changed.

## Verification

- Focused stateless tests: 25/25 passed.
- Full package test: 59 CTest suites, 2298 test cases, zero failures.
- `make autoware-build`: passed.
- Dynamic run: `output/20260831-063008` (`make dev3`).
- Registry JSON and both new manifests parse successfully. The repository's
  registry validator still stops on the pre-existing unregistered snapshot
  `000000003800-9e2c6962173873f9-...` referenced by the 20260830 Pass artifact
  experiment; this Slice did not fabricate or alter that historical evidence.

In D2 sequence 904, target progress advances at every valid stage and target
lateral remains within `-0.069477..0.207660 m`. The previous
`-1.170..-8.143 m` lateral explosion is absent. The new solve rejection is at
`steering-rate-prefix/stage=1`, not a dynamic-obstacle lateral row.

## Removed or simplified code

- `CourseProjection` and `course_projection_reason_name`.
- `project_to_recorded_course`.
- world-velocity target synthesis inside stateless maneuver generation.
- tests that encoded the second predictor as expected behavior.

## Remaining concerns

- D1 still reached `actual footprint wall margin violated` during ShiftOut.
- D2 later invalidated the locked target as stale/lost.
- The new sequence-904 steering-rate failure needs a separate frozen-snapshot
  classification; it is not evidence that the target-tube repair failed.
- This bounded run did not establish Pass/Return completion or six-lap race
  acceptance.
- The central registry has an older missing-manifest debt unrelated to this
  target-tube ownership change.

## Next dynamic acceptance item

Reproduce and classify the earliest post-repair ShiftOut failure without
changing parameters. Confirm that no new current-epoch candidate shows target
progress pinned to the wall-window endpoint or multi-metre lateral divergence.
