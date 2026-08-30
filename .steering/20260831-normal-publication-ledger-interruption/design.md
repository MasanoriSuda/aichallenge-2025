# Design: invalidate normal execution continuity at publication boundary

## Root cause

Sequence 680 was certified from the current world at decision 1279. Its
terminal Stop proof expected steering to progress from about 0.094 rad toward
0.367 rad. During the next 0.45 s, the actual output alternated between normal
Pass and external Emergency Stop. Bag data shows acceleration alternating
between `+1.33` and `-3.0 m/s^2` and different lateral commands.

The certified-plan store still resolved sequence 680 with a `PublishedPlan`
clock based on elapsed wall time since first publication. At decision 1297 it
therefore selected cursor 0.45 s although the intervening normal control
sequence was not continuously published. The expected steering was 0.3665
rad, while the actual steering was 0.0409 rad. Revalidation correctly rejected
the unreachable join, but only after the vehicle had moved from `ey=-3.11` to
`ey=-3.52`; the next Stop trajectory and actual footprint were wall-blocked.

Thus the visible wall-margin violation is downstream. The first broken
invariant is publication-ledger continuity.

## Change

Use the existing certified-plan store as the execution ledger, but clear its
executed and published-Bundle identities whenever the final published
authority is not canonical normal. This happens in
`record_final_published_authority()`, after the actual command publication, so
the invalidation is tied to observed authority rather than a proposed solver
state.

The store's certified candidate remains available. It has not been claimed as
executed; the existing evaluator treats it as a candidate and must rebuild a
current-world connection before it can regain authority. Fresh worker results
continue to replace it monotonically.

## Why this is not a fallback or lease

No alternative controller, hold duration, cooldown, timeout, or permissive
proof is added. The change removes false evidence: after a different authority
publishes, an old normal trajectory is no longer a continuously executed
trajectory. Normal authority can return only with the same proof chain already
required today.

## Rejected alternatives

- Increase wall margin: masks the late rejection and does not repair the
  skipped execution clock.
- Relax steering reachability or solver tolerance: would admit a plan whose
  expected steering was not executed.
- Add a Stop grace/lease: preserves the same false plan age behind another
  timer.
- Freeze the old wall-clock cursor: still treats a spatially changed vehicle as
  if it remained on the interrupted trajectory; current-world proof must decide
  whether a candidate is joinable.
