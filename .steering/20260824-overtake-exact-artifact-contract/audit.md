# Audit

## Observed chain

In `output/20260824-065336`, a tactical branch certificate admitted ShiftOut
with 20 exact stages. On the same control decision, the synchronous extended
solver returned a result and execution-primal normalization proceeded far
enough to call `build_exact_extended_wall_proof_input`. That builder extracted
the five-state trajectory, then failed only at
`exact_physical_execution_trajectory_complete`.

The asynchronous canonical worker was pending, but that is not sufficient
evidence that worker latency caused the rollback: the synchronous production
path already had a solved primal. The earliest proven break is the
boolean-only exact-artifact boundary.

## Competing hypotheses

1. A solved velocity is slightly negative but within its certified row
   tolerance; extraction accepts finite velocity while immutable validation
   rejects all negative values.
2. Solved progress regresses slightly within extraction tolerance; immutable
   validation requires exact nondecreasing values.
3. A different shape, finite, bound or distance invariant is lost.

No behavior change is authorized until the failed field is observed.

## Dynamic result

`output/20260824-071238` used the rebuilt binary. It admitted ShiftOut near
waypoint 160 and the synchronous exact five-state solution passed physical wall
proof. The earlier incomplete-artifact outcome did not recur, so neither
`invalid-velocity` nor `progress-regressed` is accepted as root cause.

The run did establish a persistent, earlier migration defect:

- first ShiftOut cycle: exact synchronous five-state wall proof accepted;
- canonical async worker became current-world ready on subsequent cycles;
- windows included 40/40, 38/41 and 40/41 complete canonical selections;
- production output still reported `legacy-normal-bypass`, `plan=0`, and
  `missing-canonical-command-identity` during ShiftOut and Pass;
- DynamicWait also returned to legacy normal control with unresolved canonical
  identity.

Thus the typed validator stays as diagnostic infrastructure, while its boolean
policy remains unchanged. The next behavioral Slice is the planned Overtake
authority promotion: consume the already certified canonical selection and
delete the competing normal conversion/fallback branch in that scope.
