# Results

## Implemented correction

- Maximum-braking Stop references can now be imposed directly on the
  immutable current-world seven-state snapshot without a historical normal
  execution artifact.
- Normal and Stop workers share the same immutable snapshot built after the
  actually serialized predecessor.  Their solver contexts and latest-only
  scheduling remain independent.
- Publication now owns only the live tactical scope.  It no longer submits a
  Stop solve from `selected_plan->solver_source_snapshot`.
- Exact wall, timed dynamic-obstacle and terminal-rest proof remain mandatory;
  no tolerance, clearance, horizon, timeout, lease or fallback changed.

## Verification

- `make autoware-build`: 25 packages passed.
- package CTest: 59/59 passed, including new tests that prove the current-world
  Stop preserves its control prediction origin and can build a certified
  observation without a normal artifact.
- bounded `make dev2`: `output/20260831-124927`.

Domain 1 entered ShiftOut at decision 1506.  The new worker produced current-
world results immediately; representative two-second windows consumed 36
results with 34 accepted and 30 results with 28 accepted.  Candidate-build,
solver, exact-trajectory and wall-proof rejection counts were all zero in
those windows.  The old `terminal-contingency-unavailable` failure did not
recur.

At decision 1833, the next independent failure became observable.  The
current vehicle delay prefix itself intersected the wall (`checked=23`, reject
sample 22), so both retained normal authority and the current-world Stop were
correctly rejected as `delay-prefix-blocked`.  The controller entered
Emergency Stop and the FSM reported `actual footprint wall margin violated`
about 0.12 seconds later.  This is no longer a stale Stop producer defect: the
upstream ShiftOut geometry was retained for about 1.9 seconds until the
physical state had already crossed the stoppable/wall-clear boundary.

The next Slice must freeze the earliest predecessor where this path changes
from wall-clear to delay-prefix collision and compare a fresh same-side,
opposite-side and Stop Bundle from that exact world.  It must not add another
Stop fallback or tune wall/solver parameters.

## Frozen evidence

Decision 1576 had a safe publisher interval, no front emergency and ample
reported corridor reserve.  Normal authority disappeared only because the
fixed terminal Stop hit the wall and the separately certified alternate Stop
could not join the current steering state.

The specialized snapshot comparison accepted both a free seven-state Stop
and a control-lattice Stop from the current-world problem.  This rules out
physical stop infeasibility and classifies the live failure as a producer/
scheduling identity defect.
