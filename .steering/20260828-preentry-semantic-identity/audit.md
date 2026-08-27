# Audit: Pre-entry semantic identity

## Result

The split semantic owner was repaired without changing authority, solver,
clearance, horizon, timeout, lease or fallback policy.  In
`output/20260828-005426`, the prior ShiftOut failure signature
`rate-resolved-normal-scope-inactive` did not recur.  Domain 1 completed one
canonical episode:

```text
1787846134.145 / WP190  Idle -> ShiftOut
1787846136.193 / WP196  ShiftOut -> Pass
1787846138.969 / WP201  Pass -> Return
1787846142.118 / WP210  Return -> Idle
```

The episode summary reports all three phases, 7.98 seconds elapsed, and a
canonical seven-state command contract throughout the observed transition.

## First violated invariant

Before this Slice, one speculative Overtake candidate had two semantic owners.
`init_problem()` derived its source bounds from the live Follow/Cruise intent,
while the seven-state adapter was later called with ShiftOut or Pass.  A
candidate therefore did not have one immutable intent before construction.

The repair resolves the prospective intent once at
`mpc_controller_cpp.cpp:8613` and passes that same value to both the source
problem at `mpc_controller_cpp.cpp:8632` and the seven-state builder at
`mpc_controller_cpp.cpp:8651`.  The pure resolver is defined at
`mpcc_execution_contract.cpp:104` and is covered by the four relevant entry
forms in `test_mpcc_execution_contract.cpp:16`.

## A--D classification

- Frozen A (`output/20260828-003547`, persistent Mission plus seven-state SQP):
  failed because the Mission-side hypothetical intent was not propagated to
  the source-problem producer.
- Repaired A (`output/20260828-005426`): dynamically reached ShiftOut, Pass,
  Return and Idle under the same seven-state formulation.
- B/C/D were not needed to repair this failure family.  A same-snapshot B
  execution was not available, so the formal registry classification remains
  `inconclusive`; source evidence and dynamic non-recurrence nevertheless
  identify a lifecycle/semantic-identity defect rather than candidate geometry,
  solver tolerance or physical infeasibility.

## Verification

- `make autoware-build`: 25 packages built successfully.
- Complete package CTest: 49/49 targets passed.
- Google/Python total: 1,969 tests, zero failures/errors/skips.
- Exact replay of all eight new snapshots reproduces their recorded outcomes
  in warm and cold modes.
- All eight new snapshots carry `wall-grid.bin`; the source wall contract is
  active for ShiftOut, Pass and Return.
- `git diff --check`: passed.

## Distinct successor failure

This Slice also exposed a later, independent integration failure.  The last
published Return artifact (`sequence=2398`, executed at decision 2987) lost
continuation proof at decision 2997.  At that point the vehicle was travelling
5.65 m/s with only 0.40 m measured wall distance.  Emergency braking was
correctly selected, but the vehicle crossed the wall envelope and contacted
the wall before stopping.

The new evidence does not justify a margin or braking change.  It indicates
that terminal successor viability and a physically executable contingency Stop
suffix were not proved early enough by the published Return artifact.  That
failure must be frozen and compared through the architecture escape-hatch
before another production change.
