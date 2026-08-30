# Evidence ledger

## Frozen dynamic event

Run: `output/20260830-101331`, Domain 1.

- Decision 5368 publishes certified Cruise sequence 4474.
- Decision 5370 loses its terminal Stop proof and emits one Emergency; the
  next cycle can reuse executed evidence.
- Decision 5475 loses terminal Stop proof for sequence 4547 while the proposed
  world is `progress-lift-rejected`.
- Retained telemetry reports associated normal branches as missing
  (`seq=0`, both `missing-plan`).
- Atomic admission then retains external Stop for decisions 5476--5485.
- Background normal population reaches 140.135 ms average and 392.756 ms
  maximum in the affected window, so the missing candidate-set lifecycle is
  exposed for multiple 25 ms control periods.

## Upper-system comparison

The upper log isolates asynchronous tactical branch failure from its main
GMPCC solve and continues the current solution.  The relevant architectural
property is not a looser wall margin or a solver which never fails.  It is that
one failed/late branch does not erase the sibling or global normal authority.

## Repair mapping

| Layer | Current behavior | Required behavior |
|---|---|---|
| dual producer | certifies both sides | unchanged |
| global bank | keeps latest epoch, including empty replacement | observation-only |
| certified Store | retains selected plan only | retains selected+sibling atomically |
| current-world consumer | reads newest global bank after failure | reads sibling paired with failed lifecycle entry |
| authority | exact current-world proof required | unchanged |

## Implemented ownership repair

- The certified Store now admits the selected normal-avoidance plan and its
  exact same-epoch opposite homotopy in one transaction.
- Candidate, published-Bundle-source and executed lifecycle entries retain
  their own sibling.  The pending publisher record carries that sibling, so a
  newer worker epoch cannot change the association before publication.
- The retained consumer no longer obtains execution candidates from the
  global latest branch bank.  It checks siblings associated with candidate,
  published and executed entries, in that order, with identity deduplication.
- A sibling still receives the complete current-world wall, dynamic-obstacle,
  actuation-continuity and recursive terminal-Stop proof.  Pair retention is
  not authority and does not renew any clock.
- The global branch bank remains observation-only and may continue to replace
  its snapshot with an empty newer epoch without affecting published
  lifecycle evidence.

## Static verification

- `make autoware-build`: 25 packages passed.
- Focused Store/source-contract CTest: 2/2 passed.
- Full `multi_purpose_mpc_ros` CTest: 55/55 passed.
- No solver tolerance, clearance, lease, grace period, timeout or fallback
  parameter changed.

## Dynamic verification

Run: `output/20260830-104041`, `make dev2`, about 177 race seconds.

- Episode 2 completed `ShiftOut -> Pass -> Return -> Idle` in 6.87 s.
- Episode 3 completed the same chain in 5.04 s.
- Episode 1 ended in Recovery because the current world reported
  `static wall clearance margin infeasible`; it was not reclassified as a
  sibling lifecycle failure.
- The associated sibling survived later worker epochs and was inspected in
  three telemetry windows.  In those windows both selected and sibling plans
  belonged to sequence 6626 and were correctly rejected as
  `cursor-unavailable/cursor-exhausted`.
- No `actual footprint wall margin violated` or `actual footprint intersects
  wall` message occurred.  Static contacts and stuck recovery did still occur,
  so this is not a race acceptance run.
- Domain 1 still emitted 19 normal authority-unavailable events by requested
  intent: Cruise 15, Follow 2, Return 1 and ShiftOut 1.  These cannot be fixed
  by preserving an already exhausted sibling.

## Slice conclusion

The selected-plan/sibling ownership defect is repaired: a newer empty branch
bank can no longer erase the sibling of a published lifecycle artifact.  The
dynamic run also falsifies the stronger hypothesis that this defect explains
all authority gaps.  Remaining discontinuities are dominated by producer
freshness/cursor exhaustion, and by physical wall infeasibility/contact.  They
must be audited as separate causes rather than hidden with a longer lease or a
new fallback.

Compared with `.steering/ano`, the current system can now complete individual
passes but still lacks the upper system's continuous normal authority across
late/failed tactical solves.  The next audit should freeze the first
high-speed authority gap before contact and determine why no fresh certified
current-world successor replaces the exhausted pair.
