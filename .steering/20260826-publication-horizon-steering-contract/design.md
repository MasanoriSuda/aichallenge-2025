# Design

## Root cause

The solver snapshot starts at `now + execution_prediction_delay`.  Its initial
steering is therefore a predicted physical state at the control origin.  The
solver already evaluates the certified steering-rate sequence at
`publication_interval_sec`, but this time contract is not copied into the
execution artifact.

Later, retained revalidation resolves a cursor at the current control origin
and `extract_actuation()` samples steering at `cursor.elapsed_sec`.  At a fresh
intent transition that is zero, so the future physical state is compared
directly with the last desired command over a 25 ms publication interval.
This is a time-base substitution, not a conservative clearance or rate-limit
problem.

## Selected repair

1. Seal `publication_interval_sec` into `ExecutionArtifact` at solve time.
2. Validate that it is finite, positive and no longer than the certified
   horizon.
3. Resolve publisher steering at
   `cursor.elapsed_sec + publication_interval_sec` using the existing exact
   piecewise certified-rate sampler.
4. Keep velocity, acceleration and virtual-progress extraction at the current
   execution cursor; only steering angle/curvature are endpoint commands.
5. If the endpoint is outside the remaining certified horizon, fail closed.

No post-solve steering clamp is allowed.  A later Slice may need to constrain
the first QP rate against the committed desired predecessor if the corrected
sample still fails; that must be demonstrated independently rather than folded
into this repair without evidence.

## Alternatives rejected

- Increase steering continuity tolerance: hides the time-base mismatch.
- Compare against observed steering: conflates physical state with desired
  command history.
- Keep publishing the prior intent: creates a second cross-intent owner.
- Clamp to the reachable bound: mutates the certified execution command.
