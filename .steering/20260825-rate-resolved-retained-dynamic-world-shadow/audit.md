# Root-cause Audit

## Observation

`output/20260825-061048` produced a valid retained artifact and cursor, but
every current-world attempt was rejected before physical joining because one
peer existed.  `output/20260825-061340` demonstrated the complementary case:
no peer existed, but no authoritative V2X message was supplied, so inferring
an empty world would also be unsound.

## Root cause

The retained dynamic boundary is represented by an empty-world boolean.  That
boolean conflates two independent facts: whether the world was observed, and
whether any observed peer intersects the retained trajectory.

## Rejected alternatives

- Treat `NoData` as empty: converts missing perception into authority.
- Ignore vehicles behind by current longitudinal sign: does not prove their
  future motion over the suffix.
- Use separate diagnostics and `active_vehicles()` calls: a V2X callback may
  update between calls and create a mixed-generation proof.
- Add an allow-retained-with-peers flag: hides the missing physical proof.

## Chosen correction

Snapshot one authoritative V2X generation and prove all observed peer tubes
clear of the exact retained ego trajectory.  Keep missing/invalid observations
fail-closed and keep the result shadow-only.
