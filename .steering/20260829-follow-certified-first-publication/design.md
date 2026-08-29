# Design: Follow certified-first publication

## State

The normal-worker-owned Follow population state contains only:

- target identity;
- intent generation;
- selected side sign.

It owns no trajectory, corridor, certificate, timeout or output authority.
When target or intent generation changes, the selected side is rebuilt from
the current source.

## Selection order

1. the selected side for the same target/intent;
2. the current dynamic-obstacle side in the immutable problem context;
3. the deterministic candidate order.

For each side, run the existing seven-state solver, exact wall proof and Store
admission.  Return on the first certified result.  Only a rejected preferred
side permits evaluation of the other side.

This is the single-worker equivalent of an atomic winner publication.  It does
not claim both sides are globally ranked each cycle; tactical side changes
remain a separate, non-blocking concern.

## Dynamic result

`output/20260829-103359` exercised the intended order on D1:

- the initial positive candidate was certified and published with
  `preferred=0/evaluated=1`;
- when the preferred positive side was not certified, the negative candidate
  was selected with `preferred=1/evaluated=2`;
- the following accepted negative candidate used
  `preferred=-1/evaluated=1`.

This accepts the Slice's structural hypothesis: a certified preferred side no
longer waits for the opposite side.  It does **not** accept overall Follow
production quality.  A single preferred-side solve still reached 4000
iterations in a later world state, and D1 consequently lost normal authority
and entered Stop/Recovery.  That remaining failure is downstream of candidate
ordering and must be audited as a formulation/current-world feasibility issue,
not hidden with a timeout or solver-setting change.
