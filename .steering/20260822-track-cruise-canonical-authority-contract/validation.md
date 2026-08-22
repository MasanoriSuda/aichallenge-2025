# Validation

## Static baseline findings

At baseline `f85f671`:

- `evaluate_track_cruise_shadow()` runs only after the production solve;
- a certified shadow cycle saves only `TrackCruiseShadowPendingActuation` for comparison;
- `record_solution_contract()` is called for the production formulation, not the shadow result;
- extended production failure explicitly logs `using 3-state MPCC` and runs `solve_problem()`;
- total solver failure enters `safe_failure_control()`, which decelerates and maintains/rate-limits
  steering;
- no Track/Cruise full certified executable plan store exists.

Therefore setting `progress_contouring_mpcc_overtake_only=false` or selecting the current shadow
proposal would not implement single authority. It would either disable the shadow migration path or
retain the same formulation fallback and incomplete execution provenance.

## Required deterministic cases

1. fresh current-decision five-state candidate is selected;
2. invalid fresh candidate falls to valid retained five-state candidate;
3. missing executable stages rejects an otherwise certified candidate;
4. expired candidate is rejected;
5. mismatched problem/solution identity is rejected;
6. Track/Cruise three-state candidate is rejected;
7. Follow/Pass five-state candidate is rejected in this Slice;
8. no canonical candidate resolves to Emergency Stop;
9. non-finite request time fails closed.

## Runtime assertion

This Slice must not change command behavior. Existing dynamic evidence from
`output/20260822-194818` remains the runtime baseline and must continue to report
`authority=shadow, selected=0` until promotion is separately approved.

## Failure-first result

`test_mpcc_execution_contract` failed to compile with missing
`CanonicalNormalCandidate`, `CanonicalNormalAuthorityRequest` and
`resolve_canonical_normal_authority`. This established that no equivalent canonical-only selection
contract existed at the baseline.

## Implemented contract result

- The pure selector has exactly three outputs: fresh canonical, retained canonical, Emergency Stop.
- It accepts only Track/Cruise `VelocityProgress5State` identity.
- A certificate with zero executable stages is rejected.
- A fresh solution from an older decision is rejected as fresh, while the retained path may preserve
  its original problem identity.
- Validity expiry and executable horizon consistency fail closed.
- Focused contract test: 33/33 passed.

The selector is not referenced by `mpc_controller_cpp.cpp`; runtime authority remains unchanged.

## Build and regression

- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 33/33 CTest targets passed.
- `colcon test-result --verbose`: 1,576 tests, zero errors, failures or skips.
- The existing stale `build/joycon_contract_guard/package.xml` result-parser warning remains, with
  successful command exit and no package test failure.
