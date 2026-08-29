# Task list: wall-refinement equilibrated production owner

- [x] Freeze root-cause evidence and define the problem-class owner partition.
- [x] Add the wall-refinement production solver owner.
- [x] Route every wall and coupled-wall refinement solve to that sole owner.
- [x] Keep all non-wall problem classes on the existing owner.
- [x] Add source-contract and runtime telemetry tests.
- [x] Replay the frozen wall and dynamic-only counterexamples.
- [x] Run package tests and the workspace build.
- [x] Run `make dev2` and inspect lifecycle, solver, wall and publication logs.
- [x] Record results and commit the completed Slice.

## Verification record

- `make autoware-build`: 25 packages completed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 CTest
  targets and 2123 tests passed.
- Frozen `9845010060330222052`: wall production solve accepted and unchanged
  exact wall proof accepted.
- Frozen `7896913873338064473`: coupled wall/opponent production solve accepted.
- Frozen `5862539731343104692`: dynamic-only counterexample remains rejected;
  the wall policy did not leak into this QP class.
- `make dev2`: inspected `output/20260829-143616`.  No wall-refinement
  solve rejection was observed.  Both vehicles eventually entered Recovery,
  but the causal failures were non-wall QPs: D1 exhausted its retained Follow
  cursor while a new Follow state-box QP reached 4000 iterations; D2 rejected
  a dynamic-obstacle QP.  The pre-change run `output/20260829-133704` already
  entered the same D1 Stuck path about 6.2 s after AWSIM Start, versus about
  8.1 s in the new run.  Median Track/Cruise pipeline time was effectively
  unchanged (D1 41.27 -> 40.62 ms, D2 22.98 -> 21.36 ms).

## Slice boundary

The wall numerical-class defect is closed.  The dynamic run is not accepted
as race-quality evidence: it exposes a separate Follow/dynamic-QP lifecycle
defect.  That defect must be investigated in a new root-cause Slice; it must
not be hidden here with a lease, retry, timeout, tolerance or clearance
change.
