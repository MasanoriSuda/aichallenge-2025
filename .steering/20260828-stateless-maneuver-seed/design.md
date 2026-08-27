# IM-2 stateless receding maneuver seed design

## Boundary

```text
RecordedInteractionSnapshot
  -> build_stateless_maneuver_seed(side)
       -> same seven-state semantic schema
       -> rebuilt ey/e_psi reference
       -> same wall/target/current-world evidence
       -> terminal successor intent
       -> sealed candidate Snapshot
  -> IM-3 unchanged SQP + unchanged physical proof
```

The producer is a pure function.  It has no runtime clock and no mutable
state.  A seed can therefore be regenerated from disk and compared with A on
the identical world.

## Candidate reconstruction

- State zero remains the measured equality.
- Future lateral references are rebuilt from the requested homotopy, target
  lateral prediction and required center separation.
- Each reference is clamped only to the already-recorded semantic lateral
  interval.  Feasibility is not claimed by this clamp; the unchanged dynamic
  obstacle refinement and physical proof remain authoritative.
- Future heading-offset references are reset to the course frame rather than
  copied from the Mission transition path.
- Velocity, progress, cost weights, actuator limits and stage timing are
  preserved to isolate candidate lifecycle/geometry from formulation changes.

## Terminal successor intent

When the target overlap window ends inside the horizon and the racing line is
inside the terminal wall interval, the seed names `Return`.  Otherwise it
names `Stop` only when zero terminal velocity and physical braking input are
available in the same semantic problem.  This is a pre-solve intent, not a
certificate.  IM-3 may form a `ManeuverBundle` only after the common SQP and
exact wall/opponent proofs succeed.

## Identity

The candidate keeps the source decision and observation generation, replaces
only the requested side, reseals the problem context and computes the existing
Interaction Snapshot fingerprint over the rebuilt request.  The source
interaction fingerprint is retained separately so A and B can prove they came
from one immutable input.

## Authority

The library and CLI return data only.  They do not link the ROS controller,
publisher, production adapter, certified-plan store or Recovery supervisor.
