# Validation

## Failure-first contract

Before implementation, the new physical-deletion source contract failed exactly on the retired
authority representations:

```text
1 failed, 36 passed
```

After deletion, the focused source-contract suite passed:

```text
37 passed
```

The contract rejects reintroduction of the direct controller, its latch and final-source inputs,
the direct execution formulation, and every direct-only YAML key.

## Build and package tests

- `make autoware-build`: 25 packages completed successfully.
- Test-enabled clean package build: completed successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 49/49 test targets passed.
- `colcon test-result --verbose`: 1,822 tests, zero errors, failures or skips.
- `git diff --check`: passed.
- Static search found no retired direct-authority symbol in production source or configuration.

The build emitted only the existing ROS 2 scoped-header-install recommendation and setuptools
deprecation warnings.

## Reachability and dynamic evidence

The deleted function had one definition and zero call sites. No assignment could set its active
latch, and the only producer of its historical latch lived inside that uncalled function. The final
diff therefore removes unreachable state and representations; it does not change the reachable
stopped-vehicle planning or publication graph.

The previous authority-retirement replay is recorded in
`.steering/20260823-low-speed-direct-authority-retirement/validation.md`. It observed zero
`LowSpeedDirect` publications while canonical Dynamic Escape execution continued, with no callback
overrun, actual contact, Reverse state or OvertakeLine Recovery transition in the bounded replay.
Because this Slice changes no reachable producer, a second dynamic replay would not add a distinct
acceptance claim and was not required.

## Preserved behavior

- stopped/slow V2X target confirmation;
- `LowSpeedAvoidance` behavior intent;
- gap/local-path generation and static-wall preflight;
- canonical local-corridor speed reference;
- low-speed path feedback used by bounded solver-failure crawl;
- external Emergency and Stuck/gear/reverse Recovery.

No wall margin, solver tolerance, horizon, weight, timeout or live behavior parameter was tuned.
The removed YAML keys were consumers of the unreachable direct controller only.

## Acceptance

Accepted as a behavior-neutral Slice 6 physical deletion. The retired normal-command owner can no
longer be represented, selected or configured, while stopped-vehicle avoidance continues through
the canonical MPCC contract.
