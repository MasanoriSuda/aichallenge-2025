# Requirements

## Objective

Represent the dynamic obstacle which owns stage-wise QP constraints in the
immutable seven-state problem identity, independently from the tactical
Mission target required by Follow/ShiftOut/Pass/Return semantics.

## Frozen evidence

`output/20260828-222947` and `output/20260828-223138` repeatedly lost normal
Track/Cruise authority before ShiftOut.  The latter reported 30 assembly
rejects with:

```text
physical obstacle world does not match problem identity
```

The one-second worker summary classified 38 of 81 submissions as assembly
rejects.  `resolve_dynamic_obstacle_contract()` deliberately enables a
current-target stay-behind tube for Cruise and Follow, but
`make_problem_context()` only populated `target_id` and
`target_obstacle_generation` when the semantic intent itself required a
target.  Cruise problems therefore contained physical opponent rows without
an identity for the opponent that generated those rows.

## Constraints

- Do not redefine Cruise as an Overtake Mission.
- Do not overload semantic `target_id` with a different responsibility.
- Do not relax replay-world, current-world, wall or opponent proof.
- Do not add an age lease, grace, timeout or solver tolerance.
- Preserve exact problem/solution/artifact fingerprint joins.
- Keep an inactive dynamic-obstacle constraint identity structurally empty.
- Do not commit generated run artifacts or user result files.

## Exit criteria

- Problem context has a dedicated, fingerprinted dynamic-obstacle constraint
  identity.
- Active dynamic obstacle refinement cannot be submitted without complete
  constraint identity.
- Cruise/Follow stay-behind problems carry the same obstacle generation used
  by their stage constraints.
- Replay-world validation consumes the dedicated identity and emits expected
  versus observed provenance on rejection.
- Serialization, replay, unit tests, build and package tests pass.
- A bounded `make dev2` run no longer loses all dynamic-active Cruise/Follow
  candidates because the semantic intent has no Mission target.

## Acceptance evidence

- Focused execution-contract, worker, snapshot, stateless and architecture
  comparison tests: 5/5 passed.
- Complete package suite after the schema boundary test: 2,036 tests,
  0 errors, 0 failures, 0 skipped (52/52 CTest targets).
- Workspace build: 25 packages passed.
- `output/20260828-230302`:
  - old physical-world/problem-identity rejection: 0;
  - missing dynamic-constraint identity rejection: 0;
  - canonical ShiftOut certified candidate: 3 sampled transition logs;
  - canonical ShiftOut executed-retained: 8 sampled transition logs;
  - `Idle -> ShiftOut` and normal seven-state publication observed.

The later `ShiftOut -> FollowPrepare` transition was caused by
`dynamic Mission wait: live overtake corridor unavailable`.  It is not an
identity regression and is intentionally left for a separate lifecycle Slice.
