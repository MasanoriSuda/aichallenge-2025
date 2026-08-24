# Validation

## Failure-first gate

Before production changes, `make autoware-build` failed on the new tests
because `ShadowWarmStartResolution` had no stage offset and
`shift_mpc_warm_start()` accepted no explicit advance.  This records the
missing contract rather than tuning around the runtime symptom.

## Deterministic tests

- identical geometry resolves to zero-stage advance;
- rolling geometry resolves to one-stage advance;
- a two-stage roll resolves to two-stage advance;
- zero-stage shift preserves primal and dual vectors;
- multi-stage shift moves state, input and base dual blocks by the exact
  advance and repeats only the terminal tail;
- an advance outside the previous horizon is rejected;
- incompatible geometry continues to reset.

Focused test result after the foundation/helper repair:

- `test_persistent_osqp`: 26/26 passed after the final compatibility guard;
- `test_race_mpcc_foundation`: 34/34 passed.

## Integration gates

- `make autoware-build`: passed after final controller integration;
- `multi_purpose_mpc_ros`: 40/40 CTest targets passed;
- aggregate package result: 1780 tests, 0 errors, 0 failures, 0 skipped;
- `test_persistent_osqp`: 25/25 passed;
- `test_race_mpcc_foundation`: 34/34 passed;
- source/format gate: `git diff --check` passed.

## Bounded dev2

Run: `output/20260824-172712/d1/autoware.log`.

- 0 active `ShiftOut` transitions occurred before the bounded stop;
- 3 dual-branch solutions were feasible, including one warm solve;
- 25 Overtake branch maximum-iteration failures were recorded;
- all 25 maximum-iteration failures were cold (`warm_candidate=0`);
- a Follow worker interruption showed an unchanged-geometry artifact as
  `warm_candidate=1, warm_stage_advance=0`, confirming zero-stage lineage is
  visible at runtime;
- control callback overruns remained zero in the sampled logs.

This run does not reproduce the original active-ShiftOut warm failure, so it
cannot prove that exact alignment removes that specific failure.  It does
falsify warm-stage misalignment as the sole producer of current pre-entry
solver failures: the next root-cause Slice must start from the cold QP and its
row-270 physical residual, not from retained-plan, Emergency or Recovery
symptoms and not from parameter tuning.

The older dual-branch and non-canonical extended-solver call sites retain the
existing default one-stage policy.  Migrating them requires their own geometry
identity/evidence boundary; they are not silently claimed as fixed here.

## Excluded artifact

`aichallenge/result-summary.json` is user-owned run output and must not be
staged or committed.
