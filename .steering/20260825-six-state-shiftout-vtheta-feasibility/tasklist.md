# Tasklist

- [x] Decode row 254 in solver telemetry with stage/field semantics.
- [x] Audit first-stage progress and lag dynamics/bounds.
- [x] Audit virtual-progress physical versus numerical boundary ownership.
- [x] Compare pre/post bridge runs and reject bridge causation.
- [x] Add a deterministic failure-first formulation test.
- [x] Falsify the alleged runtime interval contradiction before production changes.
- [x] Make no production correction because the hypothesis was rejected.
- [x] Run build and full package tests.
- [x] Run bounded `make dev2` diagnostic acceptance.
- [x] Update migration evidence and commit.

## Validation record

- `make autoware-build`: 25 packages succeeded.
- package CTest: 50/50 targets, 1,857 tests, zero failures/errors/skips.
- `output/20260825-235153`: semantic row and first-stage interval evidence
  collected in both domains; no empty `vtheta` interval observed.
