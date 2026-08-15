# Requirements

## Background

`output/20260815-130125` confirms that continuous Frenet-DP execution and its
atomic last-feasible handoff are active.  Recovery decreased and one complete
`Pass -> Return -> Idle` was observed, but most traffic laps still take about
80 seconds.  Runtime candidates commonly report an unknown entry/rear-clear
course role and then keep a locally feasible side until it becomes the slow
inside of a later curve.

The leading-car logs, the Pro review and the referenced MPCC implementation all
point to the same missing property: keep the current feasible solution while
evaluating course-aware alternatives over a shared progress horizon.  A curve
must not be reduced to one immutable lateral goal.

## Objective

- Keep minimum-motion passing on a straight.
- On a curve, evaluate inside-dive, outside-to-inside sweep-dive and outer-sweep
  references inside each physically available Frenet corridor.
- Express the selected tactic as a stage-wise multi-knot lateral reference so
  the existing DP can move at curve entry, apex and exit without rebuilding a
  discrete Mission at every knot.
- Preserve target, wall, lateral-acceleration and atomic-refresh hard guards.
- Preserve the last feasible execution path when a fresh tactical candidate is
  rejected.

## Constraints

- Do not replace the ROS 2 node, topic contracts or existing MPC solver.
- Do not import the external MPCC repository or add solver dependencies.
- Do not weaken wall, target-overlap, EmergencyBrake or no-return guards.
- Do not modify `aichallenge_system` or generated `output/` files.
- Keep the global acceleration limit unchanged.

## Acceptance criteria

- Straight samples select a minimum-motion reference.
- A left and right curve can generate symmetric inside/outer references.
- A sweep-dive reference contains entry, apex and exit lateral knots.
- Curve-aware guidance never leaves the supplied wall/target corridor.
- Legacy requests without course metadata retain the previous DP behavior.
- Unit tests and the package Release build pass.

## Dynamic verification

- `Frenet DP` diagnostics identify the selected tactic and knot count.
- `entry_role=unknown`, `rear_clear_role=unknown` and `rear_clear_s=inf` decrease
  for curve encounters whose DP horizon covers the target-active interval.
- `Pass -> Return -> Idle` completion increases without increasing wall aborts.
- Side/tactic changes do not chatter cycle-by-cycle.
