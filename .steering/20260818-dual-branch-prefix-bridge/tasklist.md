# Task list

- [x] Add and test the pure branch-candidate source resolver.
- [x] Connect receding/progressive prefixes to isolated extended MPCC solves.
- [x] Preserve prefix semantics through atomic entry/replacement handoff.
- [x] Add compact source/failure diagnostics.
- [x] Run focused tests and `make autoware-build`.
- [x] Record verification results.

## Dynamic confirmation left to run

- [ ] Confirm at least one `dual=L1` or `dual=R1` sample in `make dev2`.
- [ ] Confirm `source=receding-prefix` or `selected-prefix` reaches an attempted solve.
- [ ] Compare Pass/Recovery/SafetyBrake counts with `20260818-224027`.
