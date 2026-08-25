# Validation

## Failure-first evidence

Before implementation:

```text
test_uncertified_normal_failover_authorities_are_physically_deleted FAILED
detected all ten retired authority/config/API/telemetry tokens
```

After implementation, all source contracts pass:

```text
40 passed
```

## Static deletion proof

Production headers, sources, configuration and ordinary tests contain zero instances of:

- `LegacyNormalBypass`
- `SolverCrawl`
- `SolverBoundedContinuation`
- solver crawl configuration/API
- solver failure continuation API
- Dynamic Escape qualification hold API/telemetry

The failure-first deletion test retains the names as forbidden tokens.

## Build

Command:

```bash
make autoware-build
```

Result:

```text
Summary: 25 packages finished
[build_autoware] Build successful.
```

Only the existing setuptools deprecation warnings were emitted.

## Package tests

Command inside the canonical Docker build environment:

```bash
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose
```

Result:

```text
49/49 test targets passed
1816 tests, 0 errors, 0 failures, 0 skipped
```

## Diff quality

- `git diff --check`: passed.
- Runtime configuration additions: zero.
- Normal authority additions: zero.
- Deleted normal failover producers: three.

## Dynamic acceptance

A dedicated fault-injection replay is not available in this Slice. The next AWSIM run must verify:

1. normal fresh/retained canonical publication remains continuous;
2. a solver/preparation failure is logged as `authority=emergency-override`;
3. no positive-speed crawl, Dynamic Escape continuation or qualification hold appears;
4. Stuck/gear/reverse Recovery remains independent.
