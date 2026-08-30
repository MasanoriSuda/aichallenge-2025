# Design

## Observed ownership boundary

Normal Cruise/Follow/Overtake candidates are solved inside the existing
`LatestOnlyWorker` and admitted to the certified-plan Store.  Terminal Stop is
different: after retained normal authority fails, the control callback calls
`evaluate_stop_successor()`, seals a new plan and immediately attempts to join
it back to canonical authority.

Adding the lattice at that callsite would create a second synchronous fallback
and increase callback tail latency.  The target architecture is instead:

```text
immutable current-world epoch
  -> normal seven-state solve and exact proof
  -> certified normal plan enters existing Store
  -> separate latest-only Stop shadow receives the same source + normal artifact
  -> bounded Stop lattice solve and unchanged exact wall/dynamic proof
  -> observation mailbox only
```

The normal Store and sole publisher remain unchanged in this Slice.  After
dynamic evidence exists, the certified normal candidate may be extended with
an immutable terminal-successor companion.  That companion must be bound to
the exact normal artifact identity and world fingerprint; it is not an
opposite-homotopy sibling.

## First implementation boundary

Move these deterministic operations out of the audit translation unit:

1. advance an accepted normal artifact through exactly one publisher interval;
2. rebase the current-world seven-state snapshot at that boundary;
3. impose the solver-safe maximum-braking velocity law;
4. generate bounded positive/negative/hold steering-rate schedules.

The component performs no solve, proof, Store mutation or publication.  Both
the offline audit and the later live shadow consume it.

## Return limitation

The current reified Stop bundle rejects `Return`: reaching rest is not a proof
that the Return successor is semantically viable.  This Slice does not weaken
that contract.  A later promotion needs either a distinct Stop intent artifact
with an explicit successor, or a Return-specific terminal successor proof.
