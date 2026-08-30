# Evidence

## Frozen baseline

- HEAD: `44c5c791`
- Run: `output/20260830-104041/d1/autoware.log`
- Failure decision: `4120`
- Speed: about `4.66 m/s`
- Output: `canonical-normal-emergency-stop`
- Immediate reason: `rate-resolved authority unavailable`

## Causal trace

1. Cruise and Follow requests alternate near the front-distance boundary.
2. Sequence 3422, captured at decision 4051, is the last certified Cruise
   artifact visible before the event.
3. It remains current-world admissible until decision 4119.
4. At decision 4120 its progress lift is rejected.
5. The proposed Follow path reports `intent-mismatch`; no fresh Follow
   artifact joins.
6. The normal authority gap emits Emergency Stop; stuck recovery sees
   `solver_unsafe` and selects Reverse.

The worker window ending immediately after the failure reports 81 completed
evaluations and 81 additional `invalid-result` mailbox rejections, with no
consumable result.  Source inspection proves early population rejection paths
do not fill `completed_sec` or `compute_ms`, so the actual upstream reason is
discarded by the result schema before telemetry can report it.

## Validation run

- Run: `output/20260830-110056/d1/autoware.log`
- Complete overtake chains observed before the diagnostic event:
  - episode 1: `Idle -> ShiftOut -> Pass -> Return -> Idle`
  - episode 2: `Idle -> ShiftOut -> Pass -> Return -> Idle`
- Newly preserved rejection:
  - sequence 3460 / decision 4065 / Cruise
  - sequence 3541 / decision 4146 / Cruise
  - outcome: `build-rejected`
  - both normal-avoidance sides: `dynamic-target-unavailable`
  - exact detail: `target course projection unavailable at stage 0`

This converts the previous opaque mailbox `invalid-result` burst into an
immutable, consumable failure result.  It does not change command authority or
candidate acceptance.  The next behavioral Slice must address the newly
identified course-frame ownership mismatch: the physical wall frame window is
bounded by ego reachability although the stateless dynamic-obstacle candidate
also needs to project the selected target horizon.

## Verification

- `git diff --check`: passed
- `make autoware-build`: passed, 25 packages
- focused `test_mpcc_rate_resolved_shadow`: passed
- focused source-contract test: passed
- full package CTest: 55/55 passed
- `make dev2`: completed and stopped cleanly
