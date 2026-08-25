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
- [ ] Update migration Slice documentation and commit.

Static evidence:

- `make autoware-build`: passed (25 packages)
- focused MPCC/steering/authority tests: 7/7 passed
- complete `multi_purpose_mpc_ros` CTest suite: 51/51 passed
- no solver, wall-margin, speed, acceleration or steering parameter change

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
