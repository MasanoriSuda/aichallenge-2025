# Results: current-world Gate A lifecycle

## Static verification

- `make autoware-build`: passed.
- Focused Gate A resolver tests: 5/5 passed.
- Single normal authority architecture tests: 93/93 passed.
- Full `multi_purpose_mpc_ros` suite: 2,288 tests, zero errors, failures or
  skips. `colcon test-result` still reports the pre-existing stale
  `joycon_contract_guard/package.xml` lookup warning after the clean summary.

The tests fix these boundaries:

- inactive pre-entry uses only the current control epoch's feasible selected
  Mission geometry;
- absent or infeasible geometry fails closed;
- active execution never falls back to new-entry geometry;
- same-side and cross-side active replacement order is unchanged;
- the current-world geometry does not itself own a command or certificate.

## Dynamic verification

### dev2

Runs `output/20260831-024456` and `output/20260831-024700` did not exercise the
new admission path. The low-speed kart stopped near its start position and D1
used the dedicated low-speed avoidance path. They showed no new wall Recovery,
but are not counted as positive Gate A evidence.

### dev3

Run: `output/20260831-024939/d1/autoware.log`

At target `d3`:

1. the live control cycle selected a complete current-world Mission on side
   `-1` and released the generic Follow cap;
2. causal shadow sequence 1 used
   `tactical_input=current-world-preentry`;
3. the worker solved in 96.153 ms, accepted exact physical proof, and accepted
   the dynamic tube with 0.341 m minimum clearance;
4. current-world join and tactical identity both passed
   (`identity=newer-same-side`, `authority_ready=1`);
5. Gate A proposal 1 was published and `OvertakeLine` changed
   `Idle -> ShiftOut` immediately at decision 1121;
6. the certified ShiftOut current-world bundle became the canonical normal
   authority and later transitioned `ShiftOut -> Pass`.

This is the path that failed in the frozen D1 run: the new entry no longer
waits for the slower dual-worker Mission hint. The asynchronous dual solve is
still present for advisory branch comparison and active replacement.

## Remaining blocker (separate Slice)

The same dev3 episode later changed `Pass -> Recovery` at waypoint 52 with
`actual footprint wall margin violated`. It had already entered Pass and is
therefore downstream of the Gate A lifecycle defect fixed here.

The wall event must be frozen and compared separately: planned/certified wall
trajectory versus actually published trajectory and localization. It must not
be hidden by a clearance change, Recovery rule, lease or SafetyBrake
suppression in this Slice.

## Conclusion

The escape-hatch classification is confirmed dynamically:

- A (persistent Mission plus delayed dual-worker admission) missed the window;
- B (current-world receding tactical geometry plus the same causal seven-state
  SQP and certificates) entered ShiftOut and Pass;
- no candidate-generator or nonlinear-solver replacement was needed for this
  failure.

This closes the pre-entry scheduling/lifecycle defect. It does not claim
wall-contact-free or six-lap acceptance.
