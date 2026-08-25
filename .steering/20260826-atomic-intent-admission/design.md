# Design

## Observed causal chain

```text
Pass owns canonical production
  -> tactical Return wall preflight succeeds
  -> phase changes to Return
  -> synchronous six-state Return solve succeeds
  -> immutable physical proof succeeds
  -> admission logs certified=1 and drops the exact plan/result
  -> production rescans the mutable certified-plan store
  -> current-world join rejects for an unreported reason
  -> emergency command becomes the next predecessor
  -> Return solve/rejection repeats and QP conditioning collapses
```

The visible OSQP failures are downstream.  The first architectural defect is
that `RateResolvedTransitionAdmissionEvaluation::certified` conflates physical
certification with production adoption, while the ordinary current-world join
is performed later through another store lookup.

## Selected repair

`evaluate_rate_resolved_transition_admission()` owns one exact plan pointer
from solve through current-world evaluation.  Its result contains:

- solve outcome;
- immutable physical-proof outcome;
- exact certified sequence and plan;
- typed current-world retained result;
- production-authority availability.

The production owner consumes that returned retained evaluation directly.
The certified-plan store remains the lifecycle record for asynchronous and
executed plans, but it is no longer used to rediscover the synchronous
transition plan.

This does not bypass any guard.  Wall, dynamic obstacle, steering, velocity,
progress, observation and connector checks remain ordinary current-world
admission predicates.  A rejected join still emits emergency output; its
reason and blocking obstacle are no longer erased.

## Deferred work

If moving evidence shows that Return itself is committed before a joinable
plan exists, the next Slice will introduce prospective Return admission before
the tactical phase mutation.  That larger two-phase transition must be based
on the new exact rejection evidence; it must not be approximated by another
timeout or phase-specific exception.
