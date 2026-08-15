# Tasklist

- [x] Compare `20260815-110344` with the current hold/contact lifecycle.
- [x] Identify same-generation hold rearm after exhaustion.
- [x] Identify loss of Pass commit context at DynamicMissionWait entry.
- [x] Add one-shot exhausted-generation state and tests.
- [x] Carry Pass-origin forward-completion evidence into DynamicMissionWait.
- [x] Admit only classifier-approved contact into wait/prefix execution.
- [x] Add wall-bounded separation bias to the contact prefix.
- [x] Run focused tests and package build/tests.
- [x] Review interface compatibility and commit the change.

## Dynamic acceptance

- No second target-bound hold start after exhaustion in one generation.
- No `FollowPrepare -> Recovery` solely from a classifier-approved rear-side
  overlap.
- No new wall contact, frontal-contact continuation or unbounded wait.
- `Pass -> Return -> Idle` completes after body separation.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`
  - 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 25/25 test targets passed.
- `git diff --check`
  - passed.
- Interface review
  - no topic, service, message, launch, parameter or result-schema change;
  - changes are confined to `aichallenge_submit` and this steering record.

Dynamic acceptance remains a `make dev2` task because it requires another
multi-vehicle simulation run.
