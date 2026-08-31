# Results

## Structural verification

- `MissionTotalBudgetResolution` remains available as telemetry.
- The observer has no call path to Return, Recovery, retry blocking, Mission
  retention mutation, or recursive phase update.
- Telemetry is emitted only for an active, identified Mission with a finite
  elapsed time. This removed the first-run `Idle/elapsed=nan` warning noise.

## Static verification

- `test_single_authority_source_contract.py`: 99 passed.
- `make autoware-build`: passed.
- `multi_purpose_mpc_ros` package: all 59 CTest suites passed.
- Aggregated `colcon test-result`: 2323 tests, 0 errors, 0 failures.
  The pre-existing missing `joycon_contract_guard/package.xml` result warning
  was reported by the aggregate reader and did not fail this package.

## Dynamic verification

### Run 1: `output/20260831-172624`

- `Idle -> ShiftOut` at log line 1088.
- `ShiftOut -> Pass` at line 1393.
- `Pass -> Return` at line 1544.
- `Return -> Idle` at line 1597.
- No Recovery occurred in the episode.
- This run exposed invalid observer noise while no Mission was active; the
  observation-validity contract was added before the final run.

### Run 2: `output/20260831-173350`

- No `Idle/elapsed=nan` Mission-budget telemetry remained.
- No `same-target Mission total budget expired` transition occurred.
- No Mission-budget observation mutated a phase or publisher authority.
- A separate existing failure became the first terminal boundary:
  `ShiftOut -> Recovery`, reason `locked target stale or lost` at line 1257.
- Episode evidence at line 1279 reports wall/corridor minimum 6.07 m and
  maximum lateral acceleration 3.24 m/s2, so this terminal abort was not a
  wall or lateral-acceleration infeasibility.

Neither final episode remained active for 15 seconds: one completed normally
and one ended on target continuity. Therefore the old 15-second dynamic
failure was not reproduced directly. Its phase-mutation path is nevertheless
removed structurally and guarded by the authority contract test.

## Residual boundary

The next root-cause Slice must freeze the first `locked target stale or lost`
snapshot and trace:

1. raw V2X observation age and vehicle identity,
2. target continuity/reacquisition classification,
3. current-world dynamic-obstacle proof availability,
4. why a temporary target observation failure owns terminal Mission abort,
5. whether the last published certified artifact remains valid at the abort.

No stale timeout, grace period, fallback, solver tolerance, or clearance
change should be added before that provenance is established.

