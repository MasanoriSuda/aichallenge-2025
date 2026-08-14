# Requirements

## Background

Run `output/20260815-015514` confirmed that the bounded DP Pass authority and
Return reference handoff work, but every retained authority was released at
approximately 0.51--0.52 seconds while 48 m or more of the execution path
remained.  The authority age was tied to the optimizer warm-start age, so an
optimizer miss returned Pass to the legacy single-goal extension path even
when the loaded DP prefix was still physically executable.

## Objective

Keep a same-target/same-side DP Pass prefix authoritative across temporary
optimizer misses only while its executed horizon is revalidated every control
cycle against wall, lateral-acceleration and opponent constraints.

## Constraints

- Do not increase the 0.50 s optimizer warm-start age as the primary fix.
- A newly refreshed DP path must not inherit validation from the previous path.
- Actual wall contact/margin loss, emergency front risk, target discontinuity,
  invalid prediction, physical target overlap, solver recovery and forbidden
  waypoints revoke authority immediately.
- Runtime validation is a short lease, not a permanent mission latch.
- Existing ROS topics, message types, launch entry points and result schemas
  must remain unchanged.
- Preserve the user's `aichallenge/result-summary.json` modification.

## Definition of Done

- A stale optimizer source path remains authoritative when its current
  execution prefix was physically revalidated in the previous control cycle.
- The lease expires if runtime revalidation stops.
- Fresh source age and runtime validation age are observable separately.
- Focused tests, package tests and `make autoware-build` pass.
