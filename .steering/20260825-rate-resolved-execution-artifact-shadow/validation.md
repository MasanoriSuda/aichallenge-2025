# Validation

## Static checks

- `git diff --check`: PASS.
- `make autoware-build`: PASS, 25 packages.
- Full `multi_purpose_mpc_ros` test: PASS.
  - 1,839 tests;
  - 0 errors, 0 failures, 0 skipped;
  - the stale `joycon_contract_guard/package.xml` parser warning remains
    unrelated to the selected package.

## Contract coverage

- A solved shadow result contains `N + 1` exact six-state predictions and `N`
  exact acceleration/steering-rate/progress stages.
- The execution clock starts at the source snapshot, and cursor sampling
  crosses a stage boundary without changing the steering integration origin.
- Mutated identity, shape, state-zero steering, steering dynamics, lateral
  corridor and solver certificate each fail closed with a typed reason.
- Artifact and shadow headers expose no canonical plan store, normal command or
  publisher API.
- The execution artifact is a separate module from the solve/mailbox transport.

## Dynamic gate

- Commit: `619af51`.
- Command: `make dev2`.
- Artifact: `output/20260825-031820`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| aggregate windows | 6 | 55 | 61 |
| submitted | 405 | 4,381 | 4,786 |
| consumed | 382 | 4,313 | 4,695 |
| solved with valid complete artifact | 382 | 4,313 | 4,695 |
| artifact rejected | 0 | 0 | 0 |
| valid `21 state / 20 control` windows | 6 | 55 | 61 |
| maximum shadow compute | 3.258 ms | 12.960 ms | 12.960 ms |

- mailbox invalid/rollback/unsubmitted: all zero;
- all 61 aggregate records: `authority=shadow, selected=0`;
- callback windows: 247, overruns: zero, maximum: 23.273 ms;
- rate-resolved failure trace: zero;
- the only error records were the expected launch-time odometry-unavailable
  failsafe before the first observation.

## Verdict

PASS for immutable complete-artifact extraction and observation-only runtime
transport. This does **not** authorize production. The stored lateral boxes
are the exact QP boxes, but a fresh/current-world physical wall and obstacle
certificate for the rate-resolved trajectory remains the next gate.
