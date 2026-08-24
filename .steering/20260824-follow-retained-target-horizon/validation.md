# Validation

## Static validation

- `git diff --check`: passed.
- `make autoware-build`: passed; 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed;
  40/40 test targets.
- `colcon test-result --verbose`: 1801 tests, 0 errors, 0 failures,
  0 skipped. The existing stale `joycon_contract_guard/package.xml` result
  warning is unrelated to this Slice.

The tests include a current-world integration case proving both sides of the
contract:

1. an observation ending at 1.3 s cannot certify a retained execution window
   ending at 1.4 s and fails with `TargetHorizonUnavailable`;
2. covering that same current observation to 1.4 s under its recorded
   constant-velocity model produces a new fingerprint and allows the unchanged
   hard-gap proof to accept it.

## Dynamic validation

Bounded `make dev2` run:

- artifact: `output/20260824-224725`
- Domain 1 canonical Follow Emergency decisions: 10
- `target-horizon-unavailable`: 0
- physical `stage-gap-violation`: 4
- steering-continuity rejection: 4
- invalid current-origin/progress-lift at lap boundary: 2
- Domain 2 canonical Follow Emergency decisions: 0

The new log fields expose the current observation coverage and the exact
remaining retained-plan requirement as
`retained_target_horizon=available->required/extended`. The sampled status
transitions in this run did not require extension, while the pure and
integration tests exercise the extension path directly.

## Conclusion

The six nonphysical target-horizon losses observed in
`output/20260824-222801` no longer occur. This Slice did not hide or relax real
hard-gap violations. The remaining authority losses are now separated into
three independent causes and must not be addressed by changing Follow gap,
wall, steering-rate, timeout, lease, or solver parameters:

1. async canonical plan activation versus published-steering continuity;
2. lap-boundary current-origin/progress lifting;
3. actual hard-gap violation, which remains fail closed.
