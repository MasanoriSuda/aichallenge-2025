# Validation

## Static gates

- Source-contract tests: 16/16 passed.
- `make autoware-build`: 25 packages completed successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 targets,
  1,766 tests, zero errors, failures or skips.
- `git diff --check`: passed.

The source deletion gate verifies that canonical ShiftOut/Pass/Return has no
`ExitCurrentMission` runtime-wall action and that a failed optional prefix cannot
enter DynamicWait, reset line state or transition phase.

## Dynamic gate

Bounded `make dev2` run: `output/20260824-134024`.

- generation 1 entered ShiftOut at log line 605;
- its first observed command was `canonical-shiftout-retained` at line 614;
- the same generation reached `ShiftOut -> Pass` at line 660;
- no `runtime wall escape prefix unavailable` and no Mission invalidation from
  the removed family occurred;
- when canonical current-world proof later became unavailable, the explicit
  Emergency owner published `-3.0 m/s2`; the preplanner did not manufacture a
  normal command.

The exact `HoldCurrentSide` branch was not naturally reached in this run because
the encounter began at a different course position. Its semantics are covered by
the deterministic resolver and source-deletion tests. Dynamic evidence proves
the important ownership property: lack of a legacy escape prefix no longer
destroys the visible Mission identity.

## Separate next defect

This run is not a successful overtake acceptance run. Canonical ShiftOut became
unavailable first through `initial-corridor-violation` / current-origin
discontinuity. While canonical normal authority was unavailable and Emergency
had reduced actual speed to 0.87 m/s, the legacy line state still advanced to
Pass using `shift complete with fresh dynamic and physical Pass horizon`.
Canonical Pass then repeatedly returned `maximum iterations reached` with a
primal residual near 1.0, and Stuck/AWSIM Recovery eventually reset the episode.

That is a different authority-boundary defect: a phase transition became visible
before a matching executable canonical Pass artifact existed. It must be audited
in its own Slice. Solver tuning, grace, retry or restoration of the deleted
runtime-wall owner is not justified by this run.
