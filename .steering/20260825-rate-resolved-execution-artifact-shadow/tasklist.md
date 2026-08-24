# Task list

- [x] Audit the five-state canonical plan representation boundary.
- [x] Add a typed immutable six-state execution artifact.
- [x] Extract all predicted states, controls, durations and lateral boxes.
- [x] Validate full-horizon semantic steering reachability without clamping.
- [x] Add exact cursor and actuation sampling for retained-shadow evidence.
- [x] Add deterministic tests and observation-only source contract.
- [x] Run build and full package tests.
- [x] Commit the implementation (`619af51`).
- [x] Run `make dev2` and classify artifact/runtime evidence.
- [x] Update the migration map without promoting authority.

## Definition of Done

- Every `Solved` rate-resolved shadow result owns one valid immutable complete
  artifact.
- Mutated identity, shape, timing, lateral box, steering sequence or physical
  certificate is rejected with typed provenance.
- Cursor sampling crosses stage boundaries using certified steering rates.
- Runtime remains `authority=shadow, selected=0`.
