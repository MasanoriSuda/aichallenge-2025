# Tasklist

- [x] Analyze `20260815-113358` behavior/contact/hazard ordering.
- [x] Identify the DynamicMissionWait prefix/front-danger authority cycle.
- [x] Identify speed ownership being used as target identity.
- [x] Add recoverable-contact execution source to committed-corridor suppression.
- [x] Separate current/held hazard target identity from speed ownership.
- [x] Add regression tests for Pass-origin contact without a prior prefix.
- [x] Run focused tests, package build and full package tests.
- [x] Review interface compatibility.
- [x] Commit the change.

## Static verification

- `docker compose run -T --rm --no-deps autoware-build`: 25 packages built.
- Focused overtake/contact regression: 68 tests passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result`: 1085 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- No ROS topic/service/message, launch, parameter or evaluation schema change.

## Dynamic acceptance

- ContactContinuation in Pass-origin DynamicMissionWait produces a
  `contact=1` forward prefix.
- Same-target hazard diagnostics show `danger_suppress=1` while the classifier
  remains active.
- No SafetyBrake re-entry from that same target during the admitted contact.
- Different-target and unclassified-contact emergencies remain fail-closed.
