# Design: current-world proof join

## Data flow

```text
sealed Snapshot + solver artifact
  -> exact physical trajectory
  -> shared world-pose reconstruction
  -> exact wall sweep
  -> exact dynamic-obstacle sweep from the same ReplayWorld
  -> joined certificate
  -> CertifiedPlan Store
```

The joined proof is a pure rejection/certification component.  It owns no
worker, mailbox, Store, publisher, tactical state or authority.

## Slice order

1. expose the existing physical-wall Frenet-to-world reconstruction as a
   shared function and keep wall behavior unchanged;
2. move architecture replay's exact dynamic proof to the shared dynamic-proof
   component;
3. verify frozen comparison output is unchanged;
4. connect only the canonical current-world Overtake population to the joined
   proof;
5. replace one-shot dynamic-invalid admission with proof-guided SQP; do not
   retain both production branches.

## Dynamic observation

`make dev2` (`output/20260829-095750`) produced one exact current-world
Overtake candidate on d2.  The candidate passed the solver, exact wall proof
and exact dynamic proof at SQP depth 0.  It was not promoted through Gate A:
the 115.985 ms build completed 0.125 s after capture and current-world joining
correctly rejected it as `steering-unreachable`.

This is a scheduling/lifecycle observation, not evidence that the joined proof
is wrong.  The same run did not exercise a depth-greater-than-zero live
candidate.  Frozen replay remains the evidence for proof-guided SQP depth 1;
the production authority itself is unchanged.

The upper-entry `.steering/ano` log is consistent with this classification:
its main GMPCC repeatedly publishes solutions taking roughly 22--40 ms while
a separate child process owns asynchronous tactical work.  Our 116 ms
pre-entry solve is therefore too old to join in a changing steering state even
though its immutable physical proof is valid.  That latency is a later slice;
it must not be hidden by weakening the current-world join.
