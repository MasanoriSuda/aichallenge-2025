# Validation

## Static checks

- `git diff --check`: PASS
- changed Python source-contract module compiles: PASS
- `make autoware-build`: PASS
  - 25 packages completed
  - only the pre-existing setuptools deprecation warning was emitted
- full `multi_purpose_mpc_ros` package test: PASS
  - 1,837 tests
  - 0 errors, 0 failures, 0 skipped
  - the stale `joycon_contract_guard/package.xml` parser warning remains
    unrelated to the selected package

## Contract coverage

- Normal and clearly moving: safety map work is not required.
- Normal, low speed and forward intent: full safety remains required.
- Suspect/active Recovery: full safety remains required.
- solver fallback, rearm guard and dynamic lateral execution: full safety
  remains required.
- invalid speed/config values: fail closed into full safety.
- source contract proves the same eligibility result gates both preliminary
  wall classification and full safety evaluation, while
  `StuckRecoveryCore::update()` remains unconditional.

## Dynamic validation

### Run

- Commit: `10d2428`
- Command: `make dev2`
- Artifact: `output/20260825-024731`
- Duration: approximately 90 seconds after control activation

### Scheduling evidence

| Metric | Result |
|---|---:|
| telemetry windows | 189 |
| full safety evaluations | 643 |
| skipped safety evaluations | 5,570 |
| skip-only windows | 138 |
| windows containing full evaluation | 17 |
| skip-only Recovery average | 0.0164 ms |
| skip-only Recovery maximum | 0.806 ms |
| full-window Recovery average | 1.3388 ms |
| full-window Recovery maximum | 4.152 ms |
| maximum callback | 22.679 ms |
| 25 ms callback overruns | 0 |

The larger 0.806 ms skip-only maximum did not contain map safety work; it is
remaining detector/supervisor adapter work. Ordinary stable windows were
typically 0.013--0.020 ms.

### Semantic evidence

- Both domains transitioned from `race_not_started` to
  `Moving/vehicle_moving`; core updates were therefore not bypassed.
- The initial stopped/pre-race windows retained full safety evaluation.
- No rate-resolved shadow failure was emitted.
- No fatal error or callback overrun was emitted.
- Active Stuck Recovery: `NOT EXERCISED`.
- ShiftOut/Pass/Return completion: `NOT EXERCISED`.

### Verdict

PASS for the authorized Normal-state scheduling boundary. This is not an
active-Recovery or Overtake-quality acceptance run.
