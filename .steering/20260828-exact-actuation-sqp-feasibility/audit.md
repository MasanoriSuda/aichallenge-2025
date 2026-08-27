# Audit

## Root cause

The exact-bound producer reserved only one configured tolerance inside the
physical acceleration and steering-rate envelopes. `PersistentOsqpSolver`,
however, intentionally accepts OSQP `solved inaccurate` after certifying rows
with a ten-times tolerance multiplier. Sequence 1759 therefore passed the
generic QP row certificate with acceleration `1.376624 m/s^2`, then correctly
failed execution-artifact validation against the physical `1.37 m/s^2` upper
bound.

This was a producer/certificate contract split. It was not caused by wall
clearance, Mission lifetime, warm start, or the configured OSQP tolerance.

## Correction

`kSolvedInaccurateToleranceMultiplier` is now one shared solver contract.
Both residual certification and exact actuator insets use it. The physical
limits remain unchanged; the internal QP actuator interval is narrowed enough
that the largest residual accepted for `solved inaccurate` still cannot cross
the original physical boundary.

The adapter test verifies this inequality directly for both lower and upper
acceleration bounds.

## Frozen-QP classification

`classify_exact_qp.py` reconstructs the serialized physical-coordinate
`A/l/u` rows and solves a zero-objective LP with HiGHS. It changes no
production setting.

Both prior maximum-iteration snapshots were mathematically infeasible with
warm start removed:

- sequence 1150, ShiftOut wall refinement: infeasible;
- sequence 2342, Return wall refinement: infeasible.

The new Gate also produced mathematically infeasible wall/dynamic refinement
snapshots. Removing dynamic-obstacle or relevant actuation/state groups can
restore feasibility for the dynamic snapshots. These are genuine selected
candidate/refinement contradictions, not evidence for increasing solver
iterations.

## Verification

- `make autoware-build`: passed.
- Focused adapter/execution-source/persistent-OSQP tests: 3/3 passed.
- MPCC package tests: 49/49 passed with the documented tool `PYTHONPATH`.
- Classifier syntax check: passed.
- Dynamic Gate: `output/20260828-034416`.
- `invalid-acceleration-control-bounds`: 0.
- artifact construction rejection: 0.

## Dynamic Gate outcome

The exact-actuation defect is closed, but the Slice 5/6 integration Gate is
not yet accepted:

- Episode 1 ended in FollowPrepare after SafetyBrake.
- Episode 2 reached Pass and latched lateral exclusion, then ended in
  FollowPrepare after SafetyBrake.
- Pass -> Return -> Idle completions: 0/2.
- wall-refined QP rejection messages: 19.

No Mission lease, fallback, wall margin, acceleration limit, or solver
tolerance was changed. The frozen new snapshots are the inputs to the next
architecture escape-hatch comparison.
