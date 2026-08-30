# Results

## Static acceptance

- `make autoware-build`: 25 packages passed.
- package tests: 59/59 targets, 2281 tests, 0 errors/failures.
- source authority contract: 92 passed.
- `git diff --check`: passed.
- The unrelated stale `joycon_contract_guard` test-result warning remains outside this Slice.

## Dynamic acceptance

### Invalid startup run

`output/20260831-001105` is excluded. The AWSIM container disappeared around simulation time 10 s and Domain 1 subsequently reported stale odometry/failsafe behavior.

### Non-triggering but useful run

`output/20260831-001257` completed multiple Overtake episodes:

- episode 1: `Idle -> ShiftOut -> Pass -> Return -> Idle`
- episode 2: `Idle -> ShiftOut -> Pass -> Return -> Recovery`, ending in an actual static-wall intersection
- episode 3: `Idle -> ShiftOut -> Pass -> Return -> Idle`

The alternate was evaluated only at intent boundaries and was rejected with `intent-mismatch`; it was never incorrectly selected. This run also exposed a separate candidate-generation/Cruise-authority failure when every entry candidate required wall clamping. That is not attributed to the production bridge.

### Production bridge trigger

`output/20260831-001629/d1/autoware.log` reproduced the frozen authority gap.

At decision 2257:

```text
Stop lattice current-world alternate: decision=2257, intent=pass,
source=1629, normal=terminal-contingency-unavailable,
joined=1, reason=accepted, authority=canonical-normal-candidate, selected=0

Stop lattice production bridge: decision=2257, intent=pass,
ordinary=terminal-contingency-unavailable, source=1629,
join=accepted, authority=canonical-normal, selected=1
```

This proves that a Stop lattice built from the currently published Overtake source can replace an ordinary retained proof failure without creating a new authority owner. Identity-mismatched ShiftOut artifacts observed after the phase changed to Pass remained rejected.

## Classification

The frozen defect is confirmed as a missing production source-replacement lifecycle edge. It is not physical infeasibility and it does not justify a wall-margin, solver-tolerance, lease, grace-period or timeout change.

The bridge is intentionally narrow:

1. ordinary current-world authority remains first;
2. only ShiftOut/Pass may evaluate the published-source alternate;
3. the alternate must pass the unchanged current-world evaluator;
4. existing published Stop successor and external Stop remain downstream fallbacks.

## Residual failures kept out of this Slice

The dynamic run also shows that the bridge does not make the whole Pass suffix viable:

- decision 2257 spent about 45.1 ms in the retained join and the callback reached 56.2 ms;
- decision 2259 rejected the Stop suffix as `static-path-blocked` and emitted external Stop;
- decision 2260 recovered normal Pass authority;
- decision 2261 lost authority again;
- about 0.77 s after the bridge selection, the vehicle entered Recovery due to `actual footprint wall margin violated`.

These are separate timing/geometry consistency defects. They must be investigated from a frozen Pass failure snapshot rather than by widening this Slice or tuning configuration.

## Exit decision

This Slice is accepted for its stated lifecycle objective. It closes one proven authority gap and leaves the next root-cause investigation to a new Slice.
