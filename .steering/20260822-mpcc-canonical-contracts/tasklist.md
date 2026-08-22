# Slice 1 task list

## Contract and tests

- [x] Add the pure execution-contract types.
- [x] Add deterministic fingerprint implementation.
- [x] Add a failing focused unit test before production implementation.
- [x] Test context mutation and stable fingerprint behavior.
- [x] Test certificate and context mismatch rejection.
- [x] Test Emergency, Recovery, disabled and legacy-bypass decisions.

## Runtime wiring

- [x] Record the actual formulation without parsing reason strings.
- [x] Build the context from the exact `MpcProblem` sent to a solver.
- [x] Create a certificate only after solve and physical validation.
- [x] Preserve context/certificate through retained Dynamic Escape execution.
- [x] Attach `FinalControlDecision` to every final trace.
- [x] Keep final-source precedence and command math unchanged.

## Validation

- [x] `git diff --check` passes.
- [x] Focused contract and orchestrator tests pass.
- [x] `make autoware-build` passes.
- [x] Review confirms no config or output-command change.
- [x] Commit excludes `aichallenge/result-summary.json`.
- [x] Short `make dev2` trial requested with identity acceptance criteria.
