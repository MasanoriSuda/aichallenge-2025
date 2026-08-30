# Requirements

## Objective

Preserve the exact, actually-published ShiftOut execution certificate across
the tactical `ShiftOut -> Pass` boundary until canonical atomic admission
publishes a matching Pass artifact.

## Frozen evidence

- Baseline: `9a04f20d`
- Dynamic run: `output/20260830-112453/d1/autoware.log`
- At decision 2061 the tactical phase proposes Pass while canonical atomic
  admission correctly retains the previously-published ShiftOut artifact.
- The next callback stops querying that ShiftOut artifact solely because the
  tactical phase is already Pass.
- The Pass physical gate then reports no valid current-side prefix, invalidates
  the Mission, and emits an emergency Stop.

## Constraints

- Do not add a lease, grace period, timeout, fallback, solver change, clearance
  change, or parameter adjustment.
- An artifact may bridge the boundary only when the executed-plan ledger proves
  exact ShiftOut intent, target, Mission generation, side, and an available
  publication cursor.
- Do not renew artifact age or accept an unpublished candidate.
- Runtime hard-wall and current-world rejection remain authoritative.
- Preserve unrelated generated/user files.

## Definition of Done

- Tactical Pass continues to query the exact published ShiftOut artifact while
  the publisher ledger still owns it.
- Identity mismatch, exhausted cursor, changed target/generation/side, or
  missing publication still fails closed.
- Pass proof can join without a transient lateral-certificate ownership gap.
- Focused tests, full package tests/build, and `make dev2` verify the handoff.

