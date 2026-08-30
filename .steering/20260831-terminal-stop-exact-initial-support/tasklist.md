# Task list

- [x] Freeze decision 889 failure snapshot.
- [x] Separate normal continuation success from terminal proof failure.
- [x] Classify as model/certificate mismatch.
- [x] Reproduce later-sample rejection in `output/20260831-041927` and reject
      the initial-only reconciliation patch.
- [x] Demote approximate lateral support from hard wall owner to diagnostic.
- [x] Preserve exact occupancy-grid and dynamic proof as hard authorities.
- [x] Add regression tests for approximate-support false rejection and exact
      wall rejection.
- [x] Update canonical MPC integration documentation.
- [x] Run package build and full tests.
- [x] Run `make dev3` and verify approximate-support mismatch reaches exact
      wall/dynamic proof instead of causing `invalid-lateral-bounds`.
- [x] Commit only Slice-owned files.

## Definition of Done

- Frozen failure no longer dies at approximate support at any Stop sample when
  the exact footprint is clear.
- Occupied-grid Stop suffix still fails closed.
- No solver/clearance/timing parameter changes.
- Static tests pass and dynamic telemetry identifies the next authority owner.

## Verification

- Build: `make autoware-build` succeeded for 25 packages.
- Tests: package `ctest --output-on-failure` passed 59/59.
- Dynamic run: `output/20260831-043038` (`make dev3`).
- Terminal Stop `invalid-lateral-bounds`: 0 across D1/D2/D3.
- Exact terminal wall rejection remained active: D1=2, D2=1.
- Timed dynamic Stop rejection remained active with blocker identity.
- Natural approximate-support mismatch did not recur in this run; the frozen
  geometry is covered by the retained-revalidation regression test.
- Next independent failure family: normal continuation initial lateral or
  actuator/steering reachability. It is intentionally not patched here.
