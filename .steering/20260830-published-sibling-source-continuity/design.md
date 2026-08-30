# Design: published sibling source continuity

## Root cause

The Store deliberately has two different publication ledgers:

- `executed_plan_`: an unmodified artifact whose command was published;
- `published_bundle_source_plan_`: the immutable source used to build a
  stateless current-world Bundle whose proved command was published.

This distinction is correct.  A progress-rebased, stage-advanced or
latest-state-connected Bundle must not pretend that its unmodified source
artifact was executed.

The defect is in a downstream consumer.  Retained authority already tries the
published Bundle source before the older executed plan, but published
Overtake execution alignment reads only `executed_snapshot()`.  After sibling
adoption changes the tactical side, that consumer joins the new side to the
old side's artifact and rejects the source as `side-mismatch`.

## Repair

Add an atomic `latest_published_source_snapshot()` Store view.  Under the same
mutex it returns:

- the published Bundle source when present; otherwise
- the exact executed plan; otherwise
- no source.

The snapshot carries source kind, associated sibling, publication decision,
control-origin clock and source-local cursor.  Store chronology already
guarantees that a retained Bundle source is not older than exact execution;
later exact publication supersedes and clears it.

Published Overtake alignment consumes only this view.  It continues to build
and validate phase-compatible identity and publication cursor exactly as
before.  No new authority or fallback is introduced.

## Rejected alternatives

### Promote Bundle source through `mark_executed()`

Rejected because the current-world Bundle may have progress rebase, stage
advance or latest-state feedback connection.  Its source artifact was not
executed unmodified.

### Try Bundle source only after exact alignment rejects

Rejected because it leaves two consumer-owned source-selection paths and
hides the chronology invariant behind fallback behavior.

### Delay tactical side commit

Rejected because the opposite-side command has already crossed the publisher.
Keeping the old tactical side would make command and Mission disagree in the
other direction.

## Expected classification

This is a scheduling/publication-lifecycle defect: the opposite-side solve and
physical proof succeed, but the next-cycle consumer selects an older ledger.
