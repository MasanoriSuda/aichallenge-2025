# Requirements

## Objective

Replace per-evaluation thread creation in the active Overtake dual-branch
producer with one bounded persistent executor, without changing candidate
generation, solver formulation, exact certificates, tactical homotopy, or
production authority.

## Constraints

- Keep one running job at most and do not queue stale branch work.
- The outer latest-only solver worker may wait for its matching sibling; the
  40 Hz control callback may not wait for this executor.
- Reuse the existing independent negative/positive solver contexts.
- Executor failure must be explicit evidence, not an implicit sequential or
  legacy fallback.
- Do not change timeouts, leases, solver tolerances, wall margins, clearances,
  or authority selection.

## Definition of done

- A unit-tested persistent executor runs multiple jobs on one worker thread.
- Active ShiftOut/Pass no longer calls `std::async`.
- Exact same-epoch branch-bank and selected-side authority contracts remain.
- Build and all package tests pass.
- Dynamic logs still expose `selected_certified` and `sibling_certified`.
