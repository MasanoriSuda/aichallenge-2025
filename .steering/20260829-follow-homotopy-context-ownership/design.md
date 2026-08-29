# Design: Follow homotopy solver-context ownership

## Ownership

```text
normal LatestOnlyWorker
  primary context          -> persistent racing-line Follow / Track / Cruise
  follow-negative context  -> negative bounded Follow escape candidates only
  follow-positive context  -> positive bounded Follow escape candidates only
```

The contexts are numerical continuation owners, not plan Stores and not
authority producers.  Only a result that passes the existing exact wall proof,
dynamic-obstacle proof and current-world retained join can be published.

## Why this is not a new fallback

The bounded Follow population and both side candidates already exist in the
production path.  This Slice changes only the lifetime and provenance of their
OSQP/warm-start state.  It does not add another candidate, acceptance rule or
output path.

## Acceptance metrics

Compare a bounded `make dev2` run with `output/20260829-095750`:

- selected Follow escape `receding_warm_start_reason`;
- worker compute average/maximum and result age;
- consumed/current-semantic result count;
- `canonical-normal-emergency-stop` and
  `rate-resolved authority unavailable` counts;
- any cross-side semantic/provenance rejection.

If warm starts remain unavailable despite stable side/target/generation, stop
and inspect the identity contract rather than adding a time allowance.

## Dynamic result

`make dev2` produced `output/20260829-101711`.

- The first selected side was correctly bootstrapped from `empty-cache`.
- Subsequent selected negative/positive Follow escape results reported
  `available/available`; the previous per-call cold-start did not recur.
- In the early Follow windows, average pipeline compute was 34.920--58.610 ms,
  versus 64.344--194.424 ms around the comparable failure window in
  `output/20260829-095750`.
- D2 remained on canonical Cruise with only the two startup Emergency records.
- D1 still failed later.  A difficult Follow window solved both bounded sides
  serially near the iteration ceiling, increasing average compute to
  217.116 ms, maximum result age to 0.905 s and losing canonical authority.

The ownership hypothesis is accepted, but it is not the complete scheduling
fix.  The next root problem is population-level serialization: the latest-only
normal worker waits for all bounded sides before publishing one result.  A slow
losing side can therefore make an already certified winning side stale.  The
next Slice must compare atomic per-side result publication/selection; it must
not hide this with a longer retained-plan allowance.
