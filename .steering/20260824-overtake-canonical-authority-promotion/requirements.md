# Requirements

## Objective

Promote current-world-certified canonical Overtake to production for
ShiftOut, Pass and Return, while deleting the competing converted and
three-state normal authority from that intent scope.

## Pre-fix evidence

`output/20260824-072942` proved the corrected `0.40 m` wall contract:

- 74/74 fresh canonical physical proofs accepted;
- 71 current-world canonical selections completed;
- production still emitted eight `legacy-normal-bypass` decisions;
- the old converted path later failed exact wall proof and rolled ShiftOut
  into FollowPrepare.

The first production Gate, `output/20260824-074307` (Domain 1), exposed two
additional violations and was rejected:

- decision 2410 had a complete canonical five-state solution identity but the
  downstream active-overtake wall gate replaced it with
  `overtake-wall-admission-hold` and emitted `legacy-normal-bypass`;
- the callback synchronously solved the same five-state formulation whenever
  the async result was missing. Worker compute rose from the prior run's
  approximately 4.3 ms to approximately 50 ms, while duplicate callback solves
  repeatedly reached 4000 iterations and caused control overruns.

The rebuilt Gate, `output/20260824-080151` (Domain 1), proved the canonical
ShiftOut boundary itself, but exposed one remaining boundary leak and was
rejected:

- ShiftOut decisions published only certified canonical commands or explicit
  Emergency, and the old wall-hold gate did not reinterpret them;
- after the Mission changed to DynamicWait and its lateral prefix disappeared,
  canonical intent became `unknown` and the same episode fell through to the
  legacy three-state normal controller;
- that contradictory DynamicWait state must fail closed at the same authority
  boundary. Recovery remains outside this Slice.

## Constraints

- Only a complete async current-world-certified canonical selection may
  publish.
- Missing, invalid or unsafe canonical evidence must become the existing
  canonical Emergency authority.
- Overtake intents and unresolved DynamicWait states originating from an
  active Overtake episode must not fall through to extended-to-legacy conversion,
  reentry/circuit policy, three-state MPCC, legacy MPC or wall-hold normal
  authority.
- Do not add timeout, lease, grace, retry, fallback or parameter tuning.
- Preserve physical wall, obstacle, intent, target and provenance checks.
- A selected canonical trajectory's exact pose/yaw physical certificate must
  not be reinterpreted downstream through the legacy x/y tangent-yaw wall
  monitor.
- Preserve the user's generated `aichallenge/result-summary.json` change.

## Acceptance

- Source deletion gates prove every Overtake canonical intent returns before
  the old normal formulation block.
- Published ShiftOut/Pass/Return traces carry a complete five-state canonical
  problem/solution/plan identity or explicit Emergency authority.
- No ShiftOut/Pass/Return or unresolved DynamicWait trace reports
  `legacy-normal-bypass`.
- The control callback must not solve the Overtake five-state QP; the existing
  async worker remains the only solve owner.
- Static build/tests pass and a rebuilt `make dev2` exercises Overtake.
