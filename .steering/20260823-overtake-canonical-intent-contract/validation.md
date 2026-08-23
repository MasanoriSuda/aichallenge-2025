# Validation

## Static validation

- `make autoware-build`: passed, 25 packages.
- package test suite: 40/40 test programs passed.
- `colcon test-result --verbose`: 1,693 tests, zero errors/failures/skips.
- Existing missing `joycon_contract_guard/package.xml` artifact warning remains unrelated.
- `git diff --check`: passed before validation.

The tests prove:

- the exact supported/unsupported canonical normal-intent sets;
- mandatory target provenance for Follow/ShiftOut/Pass/Return;
- exact-intent acceptance for ShiftOut/Pass/Return;
- cross-intent rejection;
- canonical plan and retained-provenance rejection when target identity is absent.

## Dynamic replay

Input:

`output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`

Output:

`output/20260823-overtake-canonical-intent-replay/d1/autoware.log`

Aggregated Overtake fresh shadow:

| Stage | Count |
|---|---:|
| evaluated | 404 |
| eligible Overtake | 390 |
| complete problem context | 390 |
| lateral row contract | 353 |
| normalized exact primal | 353 |
| exact actuation/trajectory | 353 |
| swept physical certificate | 353 |
| canonical chain | 353 |
| world prediction | 353 |
| complete shadow selection | 353 |

Additional checks:

- `unsupported-intent`: 0
- canonical/direct first-actuation maximum difference: 0
- shadow maximum observed evaluation time: 1.157 ms
- `formulation=low-speed-direct`: 0
- `prediction-unavailable`: 0
- MPC callback overrun: 0

Production Overtake remains shadow-only: the replay still contains `legacy-normal-bypass` and the
existing five-state/three-state compatibility execution path. This Slice does not promote or store
the shadow plan.

## Gate conclusion

The Overtake canonical intent contract is accepted. The next Gate A blocker is 37 stage-zero
lateral-row rejections. Their violation is much larger than solver tolerance, so the next Slice
must audit initial-state and stage-zero bound identity rather than relaxing a tolerance or changing
wall clearance.
