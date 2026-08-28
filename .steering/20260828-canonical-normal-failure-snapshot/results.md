# Results

## Dynamic reproduction

`make dev2` produced run `output/20260828-171709`.  The observer recorded
three exact Follow failures without changing production authority or command
output:

| sequence | pipeline | production result |
|---:|---|---|
| 5487 | initial | maximum iterations, stage-9 velocity upper-row residual 0.0119791 |
| 5497 | wall-refinement | maximum iterations, residual 0.00940279 |
| 5575 | coupled wall/obstacle refinement | maximum iterations, residual 0.0000674 |

All three snapshots have the canonical 20-stage temporal horizon.  Warm and
cold replay reproduce the recorded 4000-iteration rejection, so the failures
are not caused by corrupt recorder state or a single bad warm start.

## Independent feasibility classification

The objective and OSQP configuration were removed and the immutable affine
constraints were replayed with SciPy HiGHS:

| sequence | affine hard set | nonlinear audit | classification |
|---:|---|---|---|
| 5487 | infeasible; minimum common slack 0.009228 m/s-class units | infeasible; minimum margin -0.01773 | constructed maneuver is physically/temporally infeasible under the recorded boxes; not an OSQP-only defect |
| 5497 | infeasible; minimum common slack 0.006914 | infeasible; minimum margin -0.01363 | wall-refined maneuver remains physically/temporally infeasible; not an OSQP-only defect |
| 5575 | feasible | near-feasible nonlinear rollout, minimum margin -0.000450 | exact QP has a solution but production OSQP does not converge; a distinct numerical/conditioning defect |

For 5487 and 5497, relaxing individual early acceleration lower bounds makes
the affine system feasible.  This is evidence that the generated velocity
envelope and reachable braking dynamics conflict; it is not evidence for
loosening acceleration or velocity limits.  The earliest invalid invariant is
that an infeasible Follow problem reaches the generic QP solver rather than
being classified during candidate/problem construction.

For 5575, HiGHS found an exact affine solution while both warm and cold OSQP
replay stopped at 4000 iterations.  This must be investigated separately from
the infeasible candidate cases.  Changing production tolerances here would
hide the first two failures and is therefore explicitly rejected.

## Runtime authority consequence

At decisions 7109 and 7119 the proposed Follow artifact was not available,
while a previously certified Cruise artifact remained current-world valid.
Atomic admission correctly retained Cruise.  The observer did not alter this
behaviour.  A later loss of both proofs may still lead to Emergency, but the
correct next action is to prevent or reject the contradictory Follow problem,
not to add cross-intent reuse, a lease, grace period or fallback.

## Verification

- `make autoware-build`: passed, 25 packages.
- focused snapshot CTest: 1/1 passed.
- full `multi_purpose_mpc_ros` test suite: 52/52 targets, 2047 tests, zero
  failures/errors/skips.
- dynamic capture: completed; the run was stopped immediately after sufficient
  evidence was frozen.

## Next root-cause slices

1. Trace 5487/5497 from the semantic velocity envelope through acceleration
   reachability and make infeasible construction fail with an explicit owner
   before QP submission.  Do not tune the bounds.
2. Independently inspect scaling/KKT conditioning for feasible snapshot 5575.
   Do not change solver settings until a specific poorly scaled or dependent
   row is identified.

The observation Slice is complete.  It does not claim that Follow interaction
replay is a ManeuverBundle A--D comparison; exact-QP completeness and tactical
interaction completeness remain separate contracts.
