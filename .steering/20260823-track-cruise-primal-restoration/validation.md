# Validation

## Failure-first

Four pure tests were added before implementation. The focused target initially
failed to compile because the restoration contract did not exist. After the
experimental implementation, all four cases passed, including infeasible
curvature box/rate intersection and matrix/linearization provenance mismatch.

## Static result

- `make autoware-build`: passed; 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: `1609 tests, 0 errors, 0 failures, 0 skipped`.

## Invalid run excluded

`output/20260823-090730` is excluded. A manual initial-pose request was sent
while the automatic orchestrator was already progressing. It replaced yaw
138.7 deg with 122.1 deg shortly before Start and caused an operator-induced
off-track/Reverse event.

## Dynamic result

Valid unattended run: `output/20260823-090945`.

- AWSIM state sequence: Spawned -> Grounded -> Ready -> Start.
- `execution-primal-restoration-reject`: 7 logged status transitions.
- legacy `execution-primal-reject`: 0, but only because the new layer consumed
  the raw defect.
- physical certificate reject: 5 logged status transitions.
- canonical Track/Cruise Emergency occurrence: 22 log matches.
- confirmed stuck: 1; Reverse maneuver: 2 log matches.
- callback overrun: 0.
- sampled restoration maximum adjustment: 3.129875.
- every sampled restoration exceeded 0.1; sampled candidates changed roughly
  120--145 fields.

## Verdict

Rejected. The dynamic acceptance condition was not met. The downstream
restoration converted a solver/formulation defect into a materially different
trajectory and increased wall/recovery risk. All source and test changes were
removed with `apply_patch`; repository behavior is back at the pre-Slice HEAD.
