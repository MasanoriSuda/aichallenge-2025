# Task list

- [x] Freeze the first current-stage-only publication and later wall rejection.
- [x] Separate the downstream stage-zero bounds symptom from the initiating defect.
- [x] Reject uncertified partial retained horizons in the revalidator.
- [x] Delete partial retained authority from the canonical contract.
- [x] Add production-adapter defense and regression tests.
- [x] Run focused tests and the source-contract suite.
- [x] Build the ROS workspace.
- [x] Run `make dev2` and inspect wall/authority transitions.
- [x] Record the dynamic acceptance result and commit.

## Verification

- Source contract: 73 passed.
- Latest rebuilt C++ tests:
  - retained revalidation: 47 passed;
  - execution contract: 74 passed;
  - command/production adapter: 11 passed.
- Full ROS build: 25 packages passed.
- Dynamic run: `output/20260829-111010`.
  - Former wall-entry sequence did not recur.
  - D1 first exposed the intended structural rejection at decision 1034:
    `terminal-contingency-unavailable`, with static full suffix clear and
    dynamic current-stage prefix only.
  - D1 then entered repeated Emergency because an exact certified Stop suffix
    is not implemented. Safety defect removal is accepted; race availability
    is explicitly not accepted.

## Next invariant

Convert the stateless bundle's non-authoritative `ContingencyStopIntent` into
an exact seven-state braking suffix and certify it against the same static wall
grid and current dynamic world. Do not re-enable current-stage authority until
that proof exists.
