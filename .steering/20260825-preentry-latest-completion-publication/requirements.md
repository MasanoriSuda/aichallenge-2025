# Requirements

## Objective

Make the selected-homotopy six-state execution worker publish every
monotonically newer completed result from the active tactical context, even
when a newer current-state snapshot is already queued.

## Root cause

The worker accepts a 40 Hz stream of current-state drafts but currently
publishes only when:

```text
completed sequence == latest submitted sequence
```

While the tactical selection remains valid, a newer snapshot normally arrives
before the 31--105 ms build/solve finishes. The completed result is therefore
discarded. A result becomes visible only after submissions stop, which in the
observed run occurred when the tactical selection became unavailable. This
transport rule creates the apparent tactical-lifetime and result-age failure;
it is not a solver, clearance or reachability threshold problem.

## Constraints

- Reuse `should_publish_latest_only_result()`; do not create a second worker
  publication policy.
- Seal and validate the tactical context epoch as well as monotonic sequence.
- Context invalidation must invalidate both the tactical and execution
  mailboxes atomically from the live controller.
- Keep the execution result observation-only. No Mission mutation, production
  store or command publication.
- No new timeout, lease, flag, fallback or parameter change.
- Do not alter solver, wall, obstacle or steering-reachability thresholds.
- Do not stage `aichallenge/result-summary.json`.

## Acceptance

- A completed result older than the newest submitted sequence may publish when
  it is newer than the last published result and belongs to the active epoch.
- A result from an invalidated epoch or a sequence rollback cannot publish.
- Source contract proves that exact latest-sequence equality is absent from
  the execution mailbox boundary.
- Build and all package tests pass.
- Bounded `make dev2` observes results while submissions continue and records
  tactical identity/current-world join outcomes without changing authority.
