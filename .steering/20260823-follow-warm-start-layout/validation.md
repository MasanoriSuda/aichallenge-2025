# Static validation

## Causal correspondence

| Observed cause | Change |
|---|---|
| Follow appends `N + 1` physical-gap dual rows | `ExtendedProgressMpcProblem` now carries an explicit typed trailing block layout |
| Base-only shifter rejects the longer Follow dual | `shift_mpc_warm_start` validates and shifts declared trailing stage-major blocks |
| Unknown future constraint rows could be shifted incorrectly | Undeclared or malformed trailing rows remain rejected |

The physical Follow inequality, solver settings, control parameters, canonical gates, and production
authority wiring are unchanged.

## Commands and results

### Build

```text
make autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

The only stderr was the existing setuptools `setup.py install` deprecation warning.

### Focused executable

```text
test_persistent_osqp
13 tests from 3 test suites
13 passed
```

This includes the two new tests:

- `ShiftsDeclaredTrailingStageBlock`
- `RejectsUndeclaredOrMalformedTrailingRows`

The pre-existing base-layout shift test also passes unchanged.

### Full package CTest

```text
38/38 tests passed
0 failed
```

## Remaining dynamic gate

Static validation proves the full Follow dual can be shifted and the package remains compatible. It does
not prove OSQP applies that warm start in a live Follow encounter. The next isolated `make dev2` evidence
run must confirm:

- `warm > 0` in `Follow MPCC shadow runtime`;
- solve/canonical-ready rate improves over the recorded 82.73% of valid-contract attempts;
- no regression at effective-gap, physical-wall, or canonical boundaries;
- every output remains `authority=shadow, selected=0`.
