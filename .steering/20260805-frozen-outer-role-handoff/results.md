# Results

## Implemented

- Added a pure frozen outer-transition goal resolver. It mirrors the admitted
  outside goal into the requested future side, clamps it to future wall bounds,
  and rejects missing role space or excessive lateral adjustment.
- Mission admission now validates the scheduled handoff as a second path from
  the old outside goal through the new outside goal, remaining Pass distance,
  and Return.
- The selected mission and runtime state retain the handoff goal and validated
  shift distance.
- Scheduled execution reuses the frozen goal instead of deriving a new goal
  from live target lateral motion.
- The behind-target crossing no longer requires 1.5 m lateral center
  separation. Fresh V2X, continuous target tracking, current and predicted
  footprint separation, wall feasibility, lateral acceleration, and target
  longitudinal clearance at shift completion remain hard gates.
- Rolling outer-role fallback also mirrors the current goal so its side label
  and physical lateral target cannot disagree.

## Static verification

- `make autoware-build`: passed; 25 packages built.
- Full `colcon test --packages-select multi_purpose_mpc_ros`: passed; 25 test
  targets, repository test summary 879 tests, zero failures/errors.
- Final focused `ctest -R ^test_v2x_overtake_core$`: passed.
- `git diff --check`: passed.

The build emitted only the existing Python `setup.py install` deprecation
warning.

## Dynamic acceptance criteria

In the next `make dev2` run, inspect `d1/autoware.log` for:

1. `PassPlan frozen` with `outer_transition=1`, finite `transition_goal`, and
   finite `transition_shift`;
2. `scheduled outer transition accepted` before the transition deadline;
3. no `no wall-feasible same-side separated goal` rejection;
4. side change followed by rear-clear, `Pass -> Return -> Idle`;
5. no contact, wall Recovery, SafetyBrake pause, or sustained hard braking
   attributable to that mission.

If admission reports `outer_transition_preflight_rejected`, the log now also
contains `outer_transition_reason`; that is a deliberate pre-ShiftOut reject
rather than a mid-Pass failure.

