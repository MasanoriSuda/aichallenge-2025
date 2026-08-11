# Task list

- [x] Analyze the latest run and identify the ownership gap.
- [x] Define bounded runtime body-clear handoff semantics.
- [x] Add core request/resolution and unit tests.
- [x] Store and consume the absolute Mission deadline in the controller.
- [x] Extend early-Pass Behavior ownership and front-danger suppression.
- [x] Resume a physically clear SafetyBrake pause directly in Pass.
- [x] Run focused unit tests.
- [x] Run package build.
- [x] Record verification results and remaining dynamic checks.

## Verification

- `make autoware-build`: passed (25 packages).
- `test_v2x_overtake_core` focused suites: 40/40 passed.
- `git diff --check`: passed.

## Dynamic check remaining

Run `make dev2` and compare against `output/20260811-203319`:

- count `Overtake -> Follow` while OvertakeLine remains ShiftOut/Pass;
- count `SafetyBrake-paused pass resumed through ShiftOut` versus direct Pass;
- count `Pass -> Return -> Idle`, SafetyBrake, Recovery, and Reverse;
- inspect `body-clear handoff entered/released` and require release on expiry or
  confirmed overlap rather than silent permanent ownership.
