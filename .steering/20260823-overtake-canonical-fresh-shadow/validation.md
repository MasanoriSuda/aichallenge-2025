# Validation

## Static validation

- `make autoware-build`: passed, 25 packages.
- package test suite: 40/40 test programs passed; 1,687 tests, zero failures/errors/skips.
- `git diff --check`: passed before the build.
- The shadow consumes the existing five-state result and performs no additional solve.
- The shadow has no connection to final command selection or publication.

## Dynamic replay

Input:

`output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`

Output log:

`output/20260823-overtake-canonical-fresh-shadow/d1/autoware.log`

Only sensor/state/V2X/trajectory topics were replayed. The recorded control command was excluded so
the current controller remained the sole publisher.

Aggregated fresh-shadow result:

| Stage | Count |
|---|---:|
| evaluated | 405 |
| eligible live Overtake | 385 |
| complete problem context | 385 |
| lateral row contract | 353 |
| normalized exact primal | 352 |
| exact actuation | 352 |
| exact trajectory | 352 |
| swept physical wall certificate | 352 |
| canonical plan/command chain | 0 |

The maximum shadow evaluation time in the one-second aggregate windows was 1.104 ms. No MPC
callback-overrun log occurred. `formulation=low-speed-direct` and `prediction-unavailable` remained
at zero, so the accepted LowSpeedDirect retirement did not regress.

## Gate A conclusion

Gate A is interpretable but not yet passed. The exact five-state execution artifact is physically
constructible on 352 replay cycles, with zero first-actuation mismatch. Every one then fails the
canonical plan contract because `canonical_normal_intent_supported()` excludes `ShiftOut`, `Pass`
and `Return`.

This Slice is accepted as diagnostic infrastructure only. Production Overtake output remains the
existing legacy-normal bypass and three-state fallback. The next Slice must extend and test the
canonical normal-intent contract for Overtake, then rerun this same shadow. It must not promote
authority or tune parameters.
