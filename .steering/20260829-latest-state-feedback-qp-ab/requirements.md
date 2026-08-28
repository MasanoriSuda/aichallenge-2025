# Requirements

## Objective

Determine whether the asynchronous seven-state MPCC candidate can be made
executable by a latest-state feedback QP, instead of projecting only the first
steering command onto the current publication envelope.

## Frozen baseline

- Commit: `7c618ffb`
- Production authority and command publication remain unchanged.
- Solver tolerances, wall clearance, dynamic-obstacle clearance, timeouts,
  leases and fallback policy remain unchanged.

## Root-cause hypothesis

The prepared artifact is rejected after asynchronous delay because its state
and input sequence belongs to the old initial condition.  Replacing only its
first steering sample creates a trajectory that was never solved as one QP.
Rebinding the already prepared seven-state problem to the latest measured
state and last actually published input, then solving one feedback QP, should
restore one internally consistent trajectory.

## Required evidence

- Unit tests prove latest x0 and serialized previous input own the feedback QP.
- The resulting artifact passes the unchanged nonlinear physical adapter.
- Runtime telemetry separates assembly, solve, artifact and downstream proof
  failures and records feedback compute time.
- The feedback result is observation-only and cannot enter the certified-plan
  store or production authority.

