# Results

## Implemented invariant

The Follow reachable velocity envelope now shares both physical owners with
the canonical seven-state problem:

1. its braking rate is the exact solver-coordinate acceleration lower bound
   whose accepted residual remains inside the physical actuator boundary;
2. its initial speed is the delay-compensated control-origin speed used by QP
   state zero.

The existing inset calculation was promoted to one public pure function and is
used by both the semantic Follow builder call and the adapter input-box build.
No tolerance, actuator bound or policy speed changed.

## Dynamic evidence

### First Gate: `output/20260828-173930`

The acceleration owner was unified, but exact snapshot 1364 exposed a second
origin mismatch.  Its QP initial speed was `7.76043 m/s`, while its first
reachable cap was `7.40071 m/s`.  With a `0.10567 s` stage and executable
minimum acceleration `-2.959596 m/s^2`, stage one could reach only
`7.44770 m/s` or higher.  The missing approximately `0.047 m/s` was the
measured-to-control prediction delta: Follow used measured speed and QP state
zero used control-origin speed.

The same invariant was extended to the temporal origin rather than adding a
cap epsilon or an authority fallback.

### Final Gate: `output/20260828-174825`

No Follow `initial` or `wall-refinement` failure with the former braking
envelope signature was recorded.  A later Follow failure occurred only after
dynamic-obstacle refinement.  Its velocity transition is independently
reachable:

- QP state-zero velocity: `3.68481 m/s`;
- first-stage duration: `0.25 s`;
- executable acceleration lower: `-2.959596 m/s^2`;
- minimum reachable stage-one velocity: `2.94491 m/s`;
- stage-one velocity upper bound: `3.51988 m/s`.

HiGHS classifies that refined problem as infeasible because the new dynamic
obstacle effective-progress rows conflict with the available motion, not
because the Follow speed envelope outruns braking.  This is a distinct next
Slice and must not be repaired by changing the work completed here.

Two single-cycle `no-current-world-authority` events remain in the final Gate:
one Follow-to-Cruise transition with the target observation unavailable, and
one Follow-to-ShiftOut transition with cursor/steering evidence unavailable.
They are not evidence that the repaired Follow problem should borrow a
cross-intent artifact.

## Verification

- `make autoware-build`: passed, 25 packages.
- focused adapter tests: 17/17 passed.
- focused Follow foundation tests: 30/30 passed.
- single-authority source contract: 67/67 passed.
- full package CTest: 52/52 targets passed.
- dynamic Gate: completed and stopped after the failure family changed.

## Exit classification

Frozen failures 5487, 5497 and 1364 were problem-construction defects:
reachable velocity state bounds were derived from a different input boundary
and/or temporal origin than the canonical QP.  The repair restores one
physical dataflow rather than masking the resulting Emergency command.

Remaining dynamic-obstacle infeasibility and independently observed feasible
QP/OSQP nonconvergence are intentionally left for separate root-cause Slices.
