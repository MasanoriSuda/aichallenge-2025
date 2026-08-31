# Design: terminal viability transition observability

Keep one diagnostic-only copy of the most recent accepted retained evaluation
for ShiftOut or Pass.  A copy is eligible only when normal production authority
exists and the current-world terminal Stop is certified.

When a terminal-contingency failure occurs, pair it only if:

- control intent is identical;
- selected source sequence is identical;
- accepted decision precedes the failed decision.

Emit one `Rate-resolved terminal viability boundary` record and append its
fields to the asynchronously persisted failure snapshot.  Update the accepted
sample only after the failure recorder has observed the current cycle, so the
sample cannot be overwritten by the failing evaluation.

This Slice deliberately does not infer or repair the cause.  It creates the
missing causal observation needed by the next root-cause Slice.

## Verification

- `make autoware-build`: 25 packages completed successfully.
- focused authority/observability contract: 97 tests passed.
- dynamic run: `output/20260831-153609/d1`.

The dynamic run did not reproduce the target same-source ShiftOut terminal
transition.  Its first Overtake episode left ShiftOut because the locked target
became stale or was lost.  A later terminal-contingency snapshot was recorded
for Cruise, outside the intentionally ShiftOut/Pass-only accepted-boundary
cache, and the vehicle subsequently entered stuck recovery.  No production
behavior was changed to force the target failure.

The next natural occurrence of the same-source ShiftOut/Pass transition will
now emit the paired record without another instrumentation change.
