# Validation

## Failure-first expectation

At baseline `1dc8ba1`, `CanonicalExecutionPlan` stores exact inputs but exports no actuation
extraction API. A test calling `extract_canonical_actuation()` must fail to compile before the
production API is added.

## Runtime status

The Track/Cruise path may compare the extracted result with its original shadow proposal, but the
final command publisher must not consume canonical actuation in this Slice.

## Implemented contract

- The extractor uses predicted state `i + 1` for target velocity and exact control stage `i` for
  acceleration, curvature and virtual-progress speed.
- Tire angle is derived only from an explicit finite positive wheelbase.
- Plan-ID mismatch, invalid remaining count, invalid stage, exhaustion and bad wheelbase fail
  closed with distinct reasons.
- The shadow evaluator compares all five actuation values with the direct primal proposal at
  `1e-12` tolerance. A mismatch cannot become `status=certified`.
- Source inspection confirms the extracted result is consumed only inside
  `evaluate_track_cruise_shadow()`; the final publisher remains unchanged and telemetry remains
  `selected=0`.

## Verification

- Canonical focused CTest: 2/2 passed.
- `make autoware-build`: 25 packages passed.
- Complete package CTest: 35/35 passed.
- `colcon test-result --verbose`: 1598 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

The pre-existing stale `build/joycon_contract_guard/package.xml` discovery warning remains
non-failing. Dynamic equality coverage remains for the next user-started AWSIM trial.
