# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `bd9fd4a refactor(mpcc): remove unproducible dynamic escape retention`
- Structural run: `output/20260825-112734`
- User-owned `aichallenge/result-summary.json` is excluded.

## Pre-fix evidence

- Both vehicle logs contain zero `Wall path admission`,
  `Dynamic escape wall`, `Dynamic escape exit`, or
  `Canonical Overtake retired legacy wall-handoff` traces.
- Source analysis proves the ActiveOvertake and DynamicEscape gate predicates
  are incompatible with the canonical normal dispatch outcomes.
- Solver handoff can still run after bounded continuation, but its incoming
  command is already a fresh canonical command with a current-world physical
  certificate.

## Failure-first proof

- Added `test_node_level_normal_wall_handoff_owners_are_physically_deleted`.
- Before implementation it failed as intended on
  `LegacyWallHandoffAuthority`, while all prior source contracts passed:
  `1 failed, 37 passed`.
- After physical deletion: `38 passed in 0.62s`.

## Static validation

- Exact-symbol search finds none of the deleted resolver, gate, exit, hold,
  final-source or publisher symbols in production headers/source or C++ tests.
- `make autoware-build`: 25 packages passed.
- Rebuilt `multi_purpose_mpc_ros` with `BUILD_TESTING=ON`: passed.
- Full package suite: 49/49 targets, 1,840 tests, zero errors, failures or
  skips.
- Canonical physical wall, executed-solution wall hold, bounded solver
  continuation, Emergency, Recovery and observation-only wall telemetry tests
  remain.

## Dynamic Acceptance

- Run: `output/20260825-124515`, `make dev2`.
- Retired `solver-wall-handoff-hold`, `overtake-wall-admission-hold`,
  `dynamic-escape-exit-wall-hold`, `Wall path admission` and legacy retirement
  traces: zero in d1 and d2.
- Canonical normal publication continued: 230 traced publications in d1 and 7
  in d2 during the bounded run; publisher mutation guard: zero.
- No solver-fallback publication was observed. Final callback telemetry at
  controlled shutdown reported zero overrun in both domains.
- The run did expose the pre-existing canonical ShiftOut Emergency when its
  async candidate was unavailable. That is a separate Overtake quality issue,
  not evidence for restoring a downstream wall owner. This Slice adds no
  fallback or threshold patch for it.
- Containers were stopped normally with `make down`.
