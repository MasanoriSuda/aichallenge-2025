# Results

## Observed mismatch

The exact nonlinear rollout and the final current-world wall proof did not
share one lateral-geometry tolerance.

- The internal physical adapter used `physical_global_tolerance`.
- That global value is based on the largest projected QP row and may be
  dominated by progress or another differently-scaled quantity.
- The final wall proof instead used
  `max(1e-5, maximum_constraint_violation + 1e-6)` metres.

Consequently an exact nonlinear trajectory could appear valid inside the
solver pipeline, skip the existing post-refinement SQP correction, and fail
only at the downstream current-world wall certificate.

## Root cause

One mixed-unit solver tolerance had two incompatible responsibilities:

1. report the global numerical acceptance envelope of the QP;
2. act as a metric lateral-wall tolerance.

The second responsibility was invalid. It hid the exact trajectory defect
from the only layer capable of relinearizing it.

## Implemented correction

`mpcc_rate_resolved_execution_artifact::physical_lateral_bound_tolerance_m()`
now owns the already-established metric contract:

```text
max(1e-5 m, maximum accepted physical row violation + 1e-6 m)
```

The same resolver is consumed by:

- immutable artifact lateral-corridor validation;
- fresh nonlinear exact trajectory construction;
- retained nonlinear continuation and its initial lateral check;
- final current-world wall proof.

`physical_global_tolerance` remains unchanged for its existing non-lateral
numerical contracts. Solver settings, wall/vehicle clearance, authority,
Mission state and fallback are unchanged.

## Frozen replay

### Decision 1566

Before this correction, the generalized wall-restoration arm solved but the
final wall proof rejected an exact lateral overshoot of about 0.001 m at
substage 236.

After the correction:

- the internal exact rollout detects the mismatch;
- the existing post-refinement SQP correction runs once;
- the fresh full refinement solves;
- nonlinear trajectory, current-world wall, dynamic-obstacle and successor
  proofs accept;
- the audit arm produces a complete `ManeuverBundle`.

The replay reports
`post_refinement_linearization=requested:1/applied:1/solved:1/count:1`
and `bundle=1`.

### Decisions 2473, 3931 and 4909

Their rebuilt final QPs still reach the unchanged OSQP 4000-iteration
contract before an artifact exists. The correction deliberately does not
accept their last iterates.

Independent HiGHS primals retain their prior classification under the unified
tolerance:

- 2473: exact nonlinear trajectory rejected at substage 244;
- 3931: complete exact proof accepted and `bundle=1`;
- 4909: dynamic proof rejects a new overlap with `d2` by 0.000379 m.

This confirms the correction fixes a real proof-boundary defect but is not a
universal backend or candidate-generation solution.

## Regression coverage

A new test sets the mixed-unit global tolerance to `0.10` while keeping the
actual row residual at `1e-8`. The physical lateral tolerance remains
`1e-5 m`, and a `1e-4 m` lateral-corridor mutation is rejected. A second
assertion verifies that a larger measured row residual, rather than the global
QP scale, controls the metric tolerance.

## Verification

- `make autoware-build`: 25 packages passed;
- focused shadow/adapter/architecture tests: 3/3 passed;
- complete `multi_purpose_mpc_ros` suite: 52/52 CTest targets, 2057 tests,
  zero errors and zero failures;
- `git diff --check`: passed;
- the only result-reader warning is the pre-existing stale generated
  `build/joycon_contract_guard/package.xml` lookup.

## Production implication

The production proof path now detects lateral nonlinear drift at the layer
which owns the existing bounded SQP correction. No new authority path was
added.

Decision 1566 still requires the separate audit-only feasibility restoration
to reach that correction from its original frozen QP, so this Slice does not
promote generalized restoration. Feasible-QP backend work and candidate wall
envelope construction remain separate follow-up questions.

## Remaining risks

- Dynamic-obstacle exact proof still has a separately demonstrated affine to
  nonlinear mismatch class.
- Several final QPs are mathematically feasible but do not converge under the
  current OSQP runtime contract.
- Decision 1161 remains physically envelope-infeasible for the selected
  candidate and must be handled in candidate/homotopy generation, not here.
