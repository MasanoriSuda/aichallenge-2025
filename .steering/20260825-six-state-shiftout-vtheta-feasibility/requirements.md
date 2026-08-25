# Requirements

## Initial objective

Identify and, only if proven, remove the earliest formulation defect behind the repeated
six-state ShiftOut `failed_iterate_row=254` cascade, without solver tuning,
source-age renewal or a five-state fallback.

## Evidence baseline

- pre execution-source repair: `output/20260825-231050`;
- post execution-source repair: `output/20260825-233538`;
- both runs use `N=20` and repeatedly fail row 254 after ShiftOut entry;
- row 254 decodes to stage-zero virtual-progress-speed input box.

## Constraints

- preserve the physical feasible set unless a producer/consumer contract is
  proven to encode a non-physical restriction;
- do not change OSQP iteration count, tolerance, weights, wall margin, speed,
  horizon or source lifetime;
- do not clamp an uncertified solve or add a retry/fallback authority;
- distinguish true primal infeasibility from poor numerical convergence;
- preserve `aichallenge/result-summary.json`.

## Final disposition

The runtime evidence rejects the initial formulation-defect hypothesis:

- row 254 is the stage-zero virtual-progress-speed input row, but its declared and
  stage-one-implied intervals remained nonempty in every observed rejection;
- failures also selected row 253 (stage-zero steering rate) and occurred in
  Track/Cruise and pre-entry, so the incident is not ShiftOut-specific;
- in the clearest Domain 1 incident, the vehicle was already at zero wall distance
  and `e_y=-2.426 m` before the later row-253/254 rejection cascade.

Accordingly this Slice is observation-only. It must not change the virtual-progress
bound, solver settings, warm start, source lifetime, wall margin or fallback policy.
The upstream wall-execution divergence is a separate root-cause Slice.

## Definition of Done

- row 254 is decoded and reported semantically;
- the exact first-stage progress/lag/input bounds and dynamics are audited;
- a failure-first test proves the diagnostic detects a real empty interval;
- runtime evidence either proves the same contradiction or falsifies it;
- no production correction is made when the runtime contradiction is absent;
- full static tests pass and bounded ShiftOut evidence no longer exhibits the
  alleged interval contradiction.
