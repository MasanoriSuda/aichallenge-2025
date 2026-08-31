# Requirements: certified Stop authority

## Objective

Prevent a certified terminal Stop contingency from masquerading as normal
ShiftOut/Pass authority.  Once that Stop command crosses the sole publisher,
the atomic intent ledger must own `Stop` until a fresh current-world normal
successor joins.

## Frozen evidence

Source:

`output/20260831-131649/d1/mpcc_architecture_snapshots/000000001609-2dae79718824ced1-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

Unchanged same-snapshot comparison found no accepted A/B/C/D forward arm:

- persistent A failed terminal Stop wall proof;
- stateless left/right B failed exact wall proof;
- rough/lattice C failed;
- offline multi-SQP D failed;
- production left/right G failed.

The independent current-world maximum-braking Stop plan did certify and was
selected by the production bridge at decision 1609.  Five callbacks later the
old normal source sequence 922 was again the retained source, then authority
failed and actual footprint wall margin was violated.

## Constraints

- do not add a resume rule, lease, grace period, timeout or fallback;
- do not change wall/opponent clearance, solver settings or vehicle limits;
- preserve the certified Stop trajectory and serialized command;
- change only its publisher/intent ownership and execution-ledger semantics;
- normal motion may resume only through the existing current-world atomic
  admission path.

## Definition of Done

- a selected certified terminal contingency publishes authority intent Stop;
- its internal source identity remains the ShiftOut/Pass world from which the
  Stop was proved;
- publication clears normal executed/published clocks and prevents the Stop
  plan from being recorded as normal execution;
- the existing atomic admission retains Stop when no fresh normal authority
  joins and admits normal immediately when one does;
- focused tests, package tests, build and bounded dynamic validation pass.
