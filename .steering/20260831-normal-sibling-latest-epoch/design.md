# Design: latest-only normal sibling worker

## Root cause

The previous Slice correctly removed the primary branch's wait on its sibling,
but kept the sibling on `BoundedSingleJobExecutor`. That executor intentionally
has no pending slot: every submission made while an older sibling is running
is rejected as `busy`.

Normal snapshots arrive faster than a branch solve. Consequently the sibling
coverage is biased toward an old epoch, while the current primary can fail
without an exact same-world opposite branch. In the frozen D1 failure the
offline B arm proves that this discarded opposite branch was certified by the
unchanged seven-state SQP.

## Change

1. Replace only the normal sibling `BoundedSingleJobExecutor` with the existing
   `LatestOnlyWorker`.
2. Submit the owned immutable sibling job through `submit_latest()`.
3. Preserve the per-epoch publication coordinator and exact-identity branch
   bank merge.
4. When a newer sibling arrives while one is running, replace only the pending
   sibling. The running solver is not interrupted.
5. Expose replacement and exception counters in the normal branch evidence
   log.

## Why this is not a fallback

No command is generated from an uncertified or older branch. The change only
ensures that optional opposite-side work converges toward the newest immutable
world instead of dropping every world received while one old solve runs. All
solver, exact wall, dynamic obstacle, certificate, Store, and current-world
admission gates remain unchanged.

## Rejected alternatives

- Wait synchronously for the opposite branch: reintroduces producer
  starvation.
- Run the opposite branch in the 40 Hz callback: violates isolation.
- Increase a retention lease or Stop grace: hides the missing producer.
- Relax wall or hard-gap constraints: unsupported by the A/B evidence.
- Queue every sibling epoch: creates unbounded stale work.
