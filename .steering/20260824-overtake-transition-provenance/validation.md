# Validation

## Static checks

- `git diff --check`: passed.
- `make autoware-build`: passed; 25 packages completed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 1,726 tests, 0 errors, 0 failures,
  0 skipped.
- Known unrelated warning remains: stale
  `build/joycon_contract_guard/package.xml` test-result reference.

The deterministic tests cover:

- exact Overtake side completeness;
- side-sensitive canonical problem fingerprinting;
- same-phase side change advancing the asynchronous epoch;
- semantic cross-side retained rejection before physical proof;
- typed cross-side current-world rejection;
- clearing a plan store without resetting monotonic stale-plan protection.

## Dynamic check

Command: `make dev2`

Artifact: `output/20260824-051821/d1/autoware.log`

The run did not enter ShiftOut, Pass or Return, so it did not exercise the new
Overtake source-separated telemetry or a live side transition. Overtake
promotion remains blocked.

Instead, the run exposed a reproducible upstream Follow defect. A retained
plan often passed current-world proof, then a subsequent observation rejected
the retained horizon at stage 19 with an approximately 1.88--2.04 m gap while
the measured current front gap remained approximately 14 m. The rejection
selected canonical Follow emergency authority and prevented Overtake entry.

## Acceptance result

- Static semantic/lifecycle contract: accepted.
- Runtime behavior change: none; shadow evidence only.
- Live old-side non-selection after a side change: not exercised.
- Production authority promotion: prohibited.
- Next root-cause target: Follow retained target-tube time/progress alignment
  at the terminal stage, not Overtake tuning.
