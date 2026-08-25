# Design

## Before

```text
canonical intent branches
  -> canonical owner or Emergency
  -> return

lexical fallthrough
  -> extended five-state synchronous solve
  -> circuit/reentry/handoff
  -> progress three-state fallback
  -> legacy spatial MPC fallback
  -> postprocessed command
```

## After

```text
Track/Cruise -> six-state canonical or Emergency
Follow       -> five-state canonical or Emergency
Shift/Pass/Return -> five-state canonical or Emergency
Rejoin       -> five-state canonical or Emergency
Stop         -> Emergency supervisor
unsupported  -> explicit canonical Emergency
```

The dispatch condition is the resolved control intent, not the migration
eligibility boolean. Eligibility remains an admission proof inside the chosen
owner. This prevents a false eligibility value from changing controller
formulation.

## Deletion boundary

- Delete the post-Rejoin synchronous normal solve block in `get_control()`.
- Delete `solve_problem()` because it has no remaining caller.
- Delete only the legacy solver state owned exclusively by that function.
- Leave common five-state builders/solvers used by Follow, Overtake and Rejoin.
- Leave Recovery and final publisher arbitration unchanged.

Migration-only telemetry and circuit/reentry objects that become unreachable
will be inventoried after compilation. They are deleted in this Slice only
when exact-use search proves they have no other responsibility.
