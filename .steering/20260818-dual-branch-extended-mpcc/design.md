# Design

## Execution boundary

The latest-only worker already owns expensive left/right Mission generation.
After generating both complete side assessments it creates an isolated MPC
snapshot for each candidate, freezes that candidate as the active Mission and
builds the normal production MPC problem using the worker observation. The two
isolated solves run concurrently inside the worker. The worker waits for both,
but the 40 Hz control callback never waits for the worker.

Each candidate is then passed through the same extended MPCC QP used by the
live controller:

- state `[e_y, e_lag, e_psi, v, theta]`;
- input `[a, kappa, v_theta]`;
- stage wall/opponent lateral bounds;
- velocity hard caps and stage/terminal velocity objectives;
- steering-rate and acceleration bounds.

The worker returns feasibility, objective value, minimum lateral bound reserve,
terminal progress/velocity and solver telemetry for both branches.

## Atomic selection

A pure selector applies this order:

1. reject unsolved, non-finite or below-reserve branches;
2. after no-return, retain the current side if it remains feasible;
3. if only one side is feasible, select it;
4. if both are feasible, select the lower objective branch;
5. retain an already committed feasible side unless the alternate objective
   improves by the configured minimum advantage.

The selected worker assessment and its complete Mission are copied together
into the existing asynchronous result.  The live callback continues to check
target, generation, phase, age and current hard faults before atomic adoption.

## Failure behavior

Branch solve failure is a soft tactical miss.  It must not clear the active
Mission, request Recovery, or make the 40 Hz callback wait.  If no branch is
selectable, the worker publishes the legacy tactical decision plus diagnostic
results; live execution keeps its current last-feasible path.
