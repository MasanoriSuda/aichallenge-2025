# Design

## Preparation / feedback split

The asynchronous full solve exports an immutable feedback preparation:

- exact final seven-state `AssemblyRequest` after wall/dynamic refinement;
- solved primal used only as a numerical seed;
- exact `Snapshot` identity, geometry and physical-world provenance.

At the latest control callback, an observation-only feedback solver:

1. replaces x0 with the latest seven-state Frenet/actuator state;
2. replaces the previous input with the last actually serialized input;
3. rebuilds x0 equality, steering prefix and first steering-rate bounds using
   the unchanged physical tolerance;
4. rolls the old input prefix through the current affine equalities as a warm
   start and solves the same final QP;
5. builds a new execution artifact and applies the existing nonlinear,
   physical-wall, dynamic-obstacle and Follow proofs.

This is an AS-RTI-style architecture experiment: preparation remains
asynchronous, while the latest-state feedback QP was temporarily measured in
the live callback.  It did not publish or store a plan.  That callback hook was
removed after the bounded experiment because the measurement itself exceeded
the 25 ms control budget.  The reusable preparation/feedback solver and its
tests remain; production integration must use a dedicated latest-only worker.

## Non-goals

- No production authority promotion.
- No new fallback, lease, grace period or timeout.
- No clearance or solver-tolerance tuning.
- No retention of a feedback trajectory merely because a Mission exists.

## Deletion milestone

Dynamic evidence showed a materially higher complete-proof rate but an
unbounded callback cost.  The next Slice must therefore atomically:

1. move feedback solve and physical certification to a dedicated latest-only
   worker;
2. promote only that feedback-certified artifact;
3. remove direct certification/adoption of the stale preparation artifact.

The temporary synchronous observation branch was deleted and may not become a
parallel owner.

## Bounded dynamic result

Run: `output/20260829-041453` (`make dev2`)

- d1: 2,860 eligible, 2,823 attempted, 1,758 complete unchanged proofs
  (62.3% of attempts); 1,065 feedback solves rejected.
- d2: 361 eligible, 330 attempted, 22 complete unchanged proofs (6.7%);
  308 feedback solves rejected.
- Every accepted feedback solve also passed the exact physical-wall and
  current-world proof in this run.
- Feedback compute maxima were 50.627 ms (d1) and 135.235 ms (d2), proving
  that synchronous callback execution is not production-safe.

Classification: the original elapsed-suffix lifecycle is a major defect
because a same-problem latest-state solve recovered many otherwise rejected
artifacts.  Remaining dual-infeasible feedback cases require separate
candidate/formulation analysis; they are not evidence for a tolerance change.
