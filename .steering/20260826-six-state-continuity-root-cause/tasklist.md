# Tasklist

## Slice A: six-state continuity

- [x] Freeze baseline HEAD and dynamic evidence boundary.
- [x] Separate the early retained-authority alternation from downstream QP
      max-iteration and Recovery symptoms.
- [x] Join rosbag command, steering report and odometry at the first incident.
- [x] Trace the retained proof time/actuation contract end to end.
- [x] Write and demonstrate a failure-first regression test.
- [x] Implement the root correction without parameter tuning or fallback.
- [x] Run focused tests, package tests and source-contract tests.
- [ ] Run moving dynamic acceptance and record evidence.
- [x] Add typed semantic-adapter diagnostics and isolate the first construction
      failure to zero-width future stop/hold intervals.
- [x] Preserve valid predicted-state and virtual-progress singleton bounds
      without weakening executable acceleration/steering-rate bounds.
- [x] Update migration Slice documentation and commit the static root fix.

Static evidence:

- `make autoware-build`: passed (25 packages)
- focused MPCC/steering/authority tests: 7/7 passed
- complete `multi_purpose_mpc_ros` CTest suite: 51/51 passed
- no solver, wall-margin, speed, acceleration or steering parameter change
- `output/20260826-075114`: first exact reject was future velocity `[0,0]`
- `output/20260826-075833`: next exact reject was virtual progress `[0,0]`
- post-correction launches: zero semantic-adapter rejects, but AWSIM stayed
  `Ready`; moving acceptance was not claimed
- `output/20260826-103853` subsequently exposed the first post-start authority
  loss at the final production boundary: decision 898 was solver/physical/world
  accepted but `joined=0,production_reason=command-rejected`.  The companion
  `.steering/20260826-certified-actuation-publication-boundary` Slice repairs
  that certified-residual interpretation mismatch; moving acceptance is still
  required before Slice A closes.

## Slice B: dynamic Overtake acceptance

- [ ] Capture ShiftOut acceptance.
- [ ] Capture direct Pass acceptance.
- [ ] Capture Pass acceptance.
- [ ] Capture Return acceptance.

## Slice C: direct Pass production migration

- [ ] Promote direct Pass Gate A to the six-state producer.
- [ ] Delete the replaced five-state tactical lifecycle in the same Slice.
- [ ] Test, dynamically accept, document and commit.

## Slice D: Slice 6 physical deletion

- [ ] Remove remaining reachable legacy/migration normal paths.
- [ ] Remove dead code and source-contract allowlists made obsolete by removal.
- [ ] Test, audit authority graph, document and commit.

## Integrated gate

- [ ] `make dev`
- [ ] `make dev2`
- [ ] `make dev3`
- [ ] no unowned normal cycle after race start
- [ ] no stale/cross-intent/cross-side artifact adoption
- [ ] no abnormal wall/QP/Recovery tail caused by authority handoff
- [ ] preserve user-owned working-tree changes
