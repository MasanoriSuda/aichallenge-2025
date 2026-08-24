# Validation

## Static validation

- `git diff --check`: passed.
- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40 test targets,
  1802 tests, zero errors, failures, or skipped tests.
- The only test-result warning was the pre-existing stale
  `joycon_contract_guard/package.xml` entry; it is outside this Slice.

## Dynamic validation

Bounded two-domain run:

- artifact: `output/20260824-230215`;
- launch: `make dev2`;
- stop: `make down` after Domain 1 crossed the circular seam twice;
- Domain 1 seam decisions used plans 4194 and 7725 near `wp_id=349`;
- Domain 1 `current origin rejected: invalid-input`: 0;
- Domain 2 `current origin rejected: invalid-input`: 0.

This satisfies the Slice objective: equivalent circular coordinates no longer
fail the retained current-origin proof.

## Residual classification

The run still contains eight Domain 1 Follow Emergency publications:

- six `canonical steering continuity rejected: unreachable`;
- two `world proof rejected: stage-gap-violation`.

Domain 2 has zero Follow Emergency publications. The steering-continuity
events recur around the two seam passages, but the initiating
`current origin rejected: invalid-input` event is gone. They therefore remain
a separate actuation/execution-cursor continuity problem and must not be hidden
by widening the circular-coordinate contract or tuning steering limits. The
stage-gap events are physical current-world proof failures and likewise remain
outside this Slice.

## Scope and artifacts

- No configuration or control limit was changed.
- `aichallenge/result-summary.json` was generated/modified by the run and is
  intentionally excluded from this Slice and its commit.
