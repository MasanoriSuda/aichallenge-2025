# Design: primary branch publication independent of sibling solve

## Root cause

`evaluate_rate_resolved_normal_avoidance_population()` evaluates both
homotopies from one immutable observation, but it currently calls
`negative_future.get()` before selecting or publishing either result. A
positive preferred branch can therefore be fully certified while remaining
invisible to the candidate Store until the negative solve and all of its proof
work finish.

The function executes in `LatestOnlyWorker`, so this does not block the 40 Hz
ROS callback directly. It still blocks the only fresh normal producer. While
that producer waits, an expired retained artifact leaves Emergency Stop as the
only valid publisher. The symptom appears as terminal-contingency loss and a
Stop window; the first broken invariant is producer availability.

## Change

1. Give normal avoidance its own persistent `BoundedSingleJobExecutor`.
2. Evaluate the current homotopy-owner preference as the primary branch in the
   normal worker.
3. Submit the opposite branch to the bounded executor using owned immutable
   source/candidate copies.
4. Publish a certified primary immediately to the candidate Store.
5. Extend the data-only normal branch bank with same-epoch `merge_branch()` so
   either completion order can attach one side without replacing the other.
6. Use a small per-epoch publication coordinator. A sibling may replace the
   candidate only when primary completion is known and primary certification
   failed. Store sequence monotonicity rejects late old-world output.
7. Return the primary evaluation without waiting for the sibling job.

## Why this is architectural rather than a fallback

No alternate controller or unproved command is introduced. The same
seven-state solver and proof pipeline certifies both branches. The change only
separates the latency of an optional sibling from the latency of the branch
that can already satisfy production admission.

## Rejected alternatives

- Increase a lease or Stop grace: hides producer starvation behind time.
- Relax terminal or wall proof: admits an unproved artifact.
- Keep `std::async` and merely store before `get()`: exposes the primary sooner
  but still blocks the latest-only producer and keeps per-cycle thread churn.
- Let the sibling overwrite the primary on completion: makes selection depend
  on timing rather than the homotopy owner.
- Retain old branch-bank pairs until both new branches finish: mixes evidence
  from different world epochs.
