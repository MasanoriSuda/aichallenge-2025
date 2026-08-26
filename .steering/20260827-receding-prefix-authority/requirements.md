# Requirements

## Objective

Remove the clean Cruise authority alternation observed in
`output/20260827-010414` without adding a legacy controller, timeout, lease,
wall-margin change, solver tuning or callback-local solve.

## Observed failure

- Decision 3853 rejected the retained plan as `continuation-wall-blocked`.
- The measured-to-control prefix was valid and clear.
- Current pose error was only 0.075 m / 0.036 rad.
- The first rejected continuation pose was about 12 m ahead, not in the
  command interval being published.
- Decisions 3853--3859 alternated certified Cruise and Emergency Stop.
- The discontinuity was followed by wall contact and Recovery.

## Root hypothesis

Retained revalidation treats the whole remaining open-loop suffix as the
authority unit. Receding-horizon execution only publishes the currently
certified prefix while a successor is solved. Discarding a wall-clear current
stage because a later suffix requires replanning creates an artificial
authority hole and destroys the state from which the successor was planned.

## Constraints

- Current measured-to-control path and current execution stage remain hard
  wall constraints.
- Dynamic-obstacle and Follow hard-gap checks remain hard constraints.
- A blocked current stage must still close normal authority.
- No parameter tuning or new normal authority producer.
- The accepted command must remain the exact command from the immutable,
  physically certified MPCC artifact.

## Definition of Done

- Failure-first tests distinguish current-stage blockage from later-suffix
  blockage.
- A later nonlinear continuation or static-wall suffix failure can retain only
  a wall-clear current MPCC stage.
- The proof and telemetry identify full-suffix versus receding-prefix scope.
- Full package tests and `make autoware-build` pass.
- A clean Track/Cruise Gate has no authority alternation caused only by a
  later continuation wall collision.
