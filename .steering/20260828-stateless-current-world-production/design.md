# Design

## Candidate population

For each already selected tactical side, rebuild from one immutable replay
world:

1. direct current-world side candidate;
2. earliest complete physical behind-to-side diagonal, when the horizon can
   contain it.

The first candidate which passes the unchanged seven-state SQP and all exact
proofs becomes the certified artifact.  The second candidate is evaluated
only when the direct candidate is not certifiable.  This bounds cost and does
not add a runtime timeout or lifecycle fallback.

## Authority boundary

```text
target + side + commit state
  -> legacy Mission semantic envelope (temporary migration input)
  -> immutable seven-state snapshot + ReplayWorld
  -> stateless current-world candidate population
  -> unchanged SQP / wall / dynamic / successor proofs
  -> existing atomic Gate A
```

The Mission trajectory is no longer solved directly.  The certified exact
trajectory remains the initial execution reference after Gate A.  Persistent
Mission deletion outside this boundary remains a later Slice.

## Evidence

- A and A2 fail the same wall-refinement row on frozen decision 3931.
- Stateless B solves but misses exact target separation by 2.37 mm.
- Physical-diagonal F has multiple fully certified schedules.
- The upper-rank log keeps a running GMPCC while a bounded ranked candidate
  population is evaluated asynchronously.

## Frozen production replay

The production-bounded G arm was added to the architecture comparator and
replayed against decision 3931 without changing solver or safety settings.

- left: no certified candidate; the best candidate remains solver-rejected;
- right/direct: not certifiable;
- right/earliest physical diagonal: accepted by SQP, exact wall proof, exact
  timed-obstacle proof and terminal-successor proof;
- accepted terminal progress: 19.8122 m;
- accepted terminal velocity: 5.47986 m/s;
- accepted minimum lateral-bound reserve: 0.111961 m.

This isolates candidate freshness/topology as the cause.  It rejects both
physical infeasibility and a solver-tolerance explanation for this snapshot.
