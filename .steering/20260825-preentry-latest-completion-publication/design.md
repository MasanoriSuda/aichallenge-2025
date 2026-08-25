# Design

## Correct latest-only semantics

`LatestOnlyWorker` replaces only a pending job. It cannot cancel the running
job. Consequently, "latest-only" means:

1. keep at most one running and one latest pending job;
2. publish every monotonically newer completion from the active context;
3. let the live consumer reject stale world/intent identity;
4. never require a completion to equal the newest queued sequence.

The tactical worker, Follow producer and unit-tested helper already use this
contract. The new execution shadow accidentally implemented a stricter and
incompatible mailbox rule.

## Context identity

Add `context_epoch` to the execution draft/result/mailbox. The draft captures
the live tactical epoch. Submission records it in the mailbox and completion
calls:

```cpp
should_publish_latest_only_result({
  result.context_epoch,
  mailbox.context_epoch,
  result.sequence,
  mailbox.latest_submitted_sequence,
  mailbox.latest_published_sequence})
```

`invalidate_mpcc_lite_async_results()` advances the shared tactical epoch and
updates both mailboxes. Thus an old running completion cannot cross a target,
phase, Mission or external-maneuver invalidation boundary.

## Authority boundary

Publication only moves an immutable shadow result into its private mailbox.
The existing live consumer still applies typed tactical identity and full
current-world revalidation. The production certified-plan store remains null,
and neither Mission nor command authority changes in this Slice.
