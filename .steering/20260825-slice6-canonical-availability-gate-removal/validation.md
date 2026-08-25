# Validation

## Failure-first evidence

Before implementation:

```text
test_canonical_normal_owner_has_no_runtime_migration_availability_switch FAILED
detected: progress_contouring_mpcc_enabled,
          progress_contouring_mpcc_overtake_only,
          progress_contouring_extended_dynamics_enabled,
          ProgressMpccDisabled,
          MigrationBoundaryInactive,
          ExtendedDynamicsDisabled,
          overtake_only_boundary
```

After implementation the same focused contract passes:

```text
1 passed, 38 deselected
```

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

The package emitted only the existing ament header-install and setuptools deprecation warnings.

## Package tests

Command (inside the canonical Docker build environment):

```bash
colcon test --packages-select multi_purpose_mpc_ros --event-handlers console_direct+
colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose
```

Result:

```text
100% tests passed, 0 tests failed out of 49
Summary: 1822 tests, 0 errors, 0 failures, 0 skipped
```

## Static deletion proof

Outside the deletion contract itself, production/config/test search returns zero instances of:

- `progress_contouring_mpcc_enabled`
- `progress_contouring_mpcc_overtake_only`
- `progress_contouring_extended_dynamics_enabled`
- `ProgressMpccDisabled`
- `MigrationBoundaryInactive`
- `ExtendedDynamicsDisabled`
- `overtake_only_boundary`

## Dynamic evidence decision

No new replay is required for this Slice.  The checked-in launch uses `config/config.yaml`, where all
three deleted switches were `true`; canonical solver construction and all reachable eligibility
branches therefore remain enabled exactly as before.  The change removes only false/missing switch
configurations that had no valid normal owner after earlier Slice 6 deletions.  Runtime numerical
parameters, physical certificates, Emergency and Recovery are unchanged.

## Acceptance

Accepted as a behavior-neutral Slice 6 structural repair.  Normal intent ownership can no longer
depend on an expired formulation migration switch.
