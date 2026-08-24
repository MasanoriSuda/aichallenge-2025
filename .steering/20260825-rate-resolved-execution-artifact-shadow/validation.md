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

Pending committed `make dev2` evidence. Required evidence is zero artifact
rejection across solved Track/Cruise windows with
`artifact_valid=1/states=N+1/controls=N` and unchanged
`authority=shadow, selected=0`.
