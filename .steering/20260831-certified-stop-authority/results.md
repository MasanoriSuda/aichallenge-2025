# Results: certified Stop authority

## Root-cause classification

Frozen source:

`output/20260831-131649/d1/mpcc_architecture_snapshots/000000001609-2dae79718824ced1-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

The unchanged architecture comparison was rerun after the implementation.
Persistent A rejected its terminal Stop wall proof; stateless left/right B,
rough/lattice C, offline multi-SQP D and production left/right all failed to
obtain forward authority.  The physical no-escape classification therefore
did not change.

The live current-world Stop lattice had independently certified a
maximum-braking trajectory at decision 1609.  The old bridge published that
trajectory with `ShiftOut` authority, left the normal execution ledger live,
and allowed an older normal source to reappear five callbacks later.  The
root cause was publisher authority identity, not Stop generation or forward
candidate quality.

## Structural correction

- A selected terminal contingency carries an explicit marker through retained
  evaluation.
- A pure contract resolver maps a valid normal source plus that marker to
  external `Stop` authority.
- Internal problem/solution/command identity remains the original
  ShiftOut/Pass provenance.
- Stop publication cannot promote the plan to normal executed evidence,
  record a published normal Bundle source, commit a sibling token, or seed a
  subsequent Stop lattice.
- The final publisher records `Stop`; the existing atomic admission therefore
  requires a fresh current-world normal join before normal execution resumes.

No lease, timeout, grace, fallback, clearance, solver tolerance or vehicle
limit was added or changed.

## Verification

- Host ownership test: 95/95 passed.
- `make autoware-build`: 25/25 packages succeeded.
- Explicit test-target build succeeded.
- Final package CTest: 59/59 passed, with zero failures (the same suite also
  passed during the preceding explicit test-target verification).
- Frozen decision 1609 comparison retained the all-forward-arms-failed
  classification.
- Bounded dynamic run: `output/20260831-134900` (`make dev2`).

The bounded run selected the certified terminal-contingency bridge on D2 at
decisions 3552, 3554 and 3558.  Each joined published Stop successor reported
`authority=certified-stop` and immediately interrupted the normal execution
ledger.  Decisions 3559 through 3576 then reported `previous=stop`,
`effective=stop`, `previous-retained` while the proposed Cruise continuation
failed current-world proof.  A fresh normal solve later became available;
decision 3587 published a production Cruise command.  No old normal artifact
was replayed without proof.

## Residual family

The same bounded run contains separate, longer Emergency Stop intervals where
the proposed normal intent repeatedly fails current-world continuation or
steering reachability before a normal successor joins.  That is not a reason
to undo Stop authority semantics or add a resume timeout.  The first such
failure must be frozen and classified independently in the next Slice.
