# Task list

- [x] Reconfirm root-cause boundary and preserve dirty user artifact.
- [x] Add typed rate-resolved runtime shadow evaluator and mailbox.
- [x] Materialize the exact semantic request inside the five-state builder.
- [x] Submit Track/Cruise snapshots through a latest-only worker.
- [x] Add aggregate identity/age/timing/publishability telemetry.
- [x] Add deterministic worker/mailbox/solver tests.
- [x] Run focused tests, full package tests and package build.
- [x] Audit that no six-state result reaches production selection/publication.
- [x] Commit the static Slice (`9697d35`).
- [x] Run `make dev2`, analyze evidence and update validation.

## Definition of Done

- Zero new normal-authority branches or configuration flags.
- Six-state results are observable but unexecutable.
- Source identity and sequence are retained across the worker boundary.
- A malformed adapter/assembly/solve/sample is typed, not hidden by fallback.
- Static and dynamic evidence is recorded in this steering directory.
