# Validation

## Static validation

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: 33/33 targets passed.
- `colcon test-result --verbose`: 1,552 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- The unrelated user change `aichallenge/result-summary.json` was not modified or staged.

## Dynamic comparisons

All runs used `make dev`, approximately four waypoint wraps, Track/Cruise shadow authority only.

| Run | Formulation | Certified / eligible | Solver max | Shadow max | Callback max | Overruns | selected=1 |
|---|---|---:|---:|---:|---:|---:|---:|
| `20260822-142549` | exact-heading post-certificate baseline | 7427/7505 (98.96%) | 11.807 ms | 14.939 ms | 30.061 ms | 4 | 0 |
| `20260822-151823` | conditional second solve | 7621/7708 (98.87%) | 38.550 ms | 46.449 ms | 53.942 ms | 60 | 0 |
| `20260822-153355` | fixed-heading `e_y` preflight | 7581/7700 (98.45%) | 22.715 ms | 25.038 ms | 41.920 ms | 59 | 0 |
| `20260822-155509` | linearized `e_y/e_psi` preflight | 7024/7124 (98.60%) | 22.991 ms | 60.859 ms | 78.254 ms | 56 | 0 |
| `20260822-161428` | accepted row-contract only | 7627/7662 (99.54%) | 10.044 ms | 12.350 ms | 29.771 ms | 1 | 0 |

The rejected formulations reduced some final hard-contact counts but increased missing solver
results, constraint rejects, and 40 Hz overruns. They did not produce a conservative physical proof.

## Final run details

`output/20260822-161428`:

- solve result present: 7,662 / 7,662;
- lateral semantic contract: 7,662 / 7,662;
- physical certificate: 7,627 / 7,662;
- final physical rejects: 26 hard contacts, 9 swept violations;
- Track/Cruise shadow authority selected: 0;
- observed outcome statuses: certified and physical-certificate-reject only.

No dynamic lateral semantic rejection occurred in this run, but the mixed-unit unit test proves the
failure boundary deterministically. The semantic contract is retained because future changed
geometry or solver-inaccurate results must not rely on a globally scaled tolerance.

## Decision

Accept the row-specific constraint provenance and lateral semantic gate. Reject and delete all three
physical preflight/refinement implementations. Do not promote Track/Cruise authority yet: exact-pose
physical rejects remain and Slice 2's zero-certificate-failure exit gate is not satisfied.
