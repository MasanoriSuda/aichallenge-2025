# Design: canonical bootstrap clock contract

Replace the ambiguous unpublished clock with three explicit lifecycle kinds:

- `BootstrapCandidate`: no executed predecessor exists; cursor is exactly zero;
- `TimeAlignedCandidate`: an executed predecessor exists; select the suffix at
  the current control origin and prove the join;
- `PublishedPlan`: advance from the artifact-local cursor recorded at the
  first exact publisher join.

The certified-plan Store already exposes candidate and executed snapshots.
The normal consumer therefore selects the clock from immutable lifecycle
evidence instead of time, age, intent or a new flag:

```text
candidate == executed       -> PublishedPlan
candidate != executed,
executed exists             -> TimeAlignedCandidate
candidate exists,
executed absent             -> BootstrapCandidate
```

Pre-entry and Gate-A proposals are moving tactical successors and remain
time-aligned candidates. They do not receive bootstrap semantics.

This is a clock-ownership correction, not a command projection. The existing
current-world proof remains the sole path from a candidate to production.
