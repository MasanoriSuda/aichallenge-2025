# Requirements: normal authority-loss snapshot

## Objective

Freeze the first race-session normal-authority loss in
`output/20260831-190352/d1/autoware.log` before changing production control.
Decision 1150 changed from retained Cruise to external Emergency Stop while
the requested intent was Follow and ego was travelling about 3.70 m/s.

## Frozen evidence

- Decisions 1113--1149 requested Follow but retained the preceding certified
  Cruise artifact because the proposed Follow artifact was not yet joinable.
- Decision 1150 had no current Follow authority and no longer had joinable
  preceding Cruise authority, so Emergency Stop was published.
- The next telemetry window contained 80 Follow submissions, 72 pending
  replacements, seven solve rejections and one certified Follow result.
- That accepted result belonged to decision 1167, after the Stop had already
  changed the tactical state; it could not repair decision 1150.
- Existing architecture snapshot capture is restricted to terminal
  contingency failures, so this first failure cannot yet be replayed through
  the A/B/C/D comparison.

## Required change

- Capture the immutable current-world seven-state interaction only when the
  final normal admission has no certified authority and will publish external
  Emergency Stop.
- Reuse the exact problem, physical wall, obstacle, actuation and serialized
  predecessor provenance already required by the architecture audit.
- Record asynchronously and rely on the recorder's semantic/homotopy
  deduplication.
- Do not solve, store, publish, retain or alter a candidate in this Slice.

## Prohibited changes

- No Mission resume rule, lease, grace period, timeout or fallback.
- No solver setting, wall clearance, vehicle clearance or speed change.
- No production authority or intent-transition behavior change.
- No classification from logs alone; A/B/C/D classification requires the
  replay-ready snapshot.

## Exit

The next bounded `make dev2` run must write a replay-ready Follow authority
loss snapshot.  That frozen snapshot is then evaluated as:

- A fails, B succeeds: lifecycle/scheduling defect;
- A/B fail, C succeeds: candidate generation defect;
- A/B/C fail, D succeeds: single-SQP limitation;
- all fail: physical infeasibility;
- solve succeeds but proof fails: model/certificate mismatch.
