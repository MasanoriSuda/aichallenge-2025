# Requirements

## Objective

Physically delete the retired `LowSpeedDirect` normal-command authority and its private
compatibility state after production authority was removed in `a3929d7` follow-up Slice 4 work.
Stopped-vehicle handling must continue to enter the canonical MPCC as corridor, target and speed
constraints.

## Repaired invariant

Every normal racing command is produced by a certified canonical MPCC solution. A normal authority
which has no producer must not remain representable in controller state, final-source arbitration,
execution-contract formulation or runtime configuration.

## Earliest violation

`low_speed_shift_control()` has no call site and no assignment can set
`low_speed_shift_control_active_` to true, but its latch, phase machine, retained-pass controller,
publisher overrides and `LowSpeedDirect`/`LowSpeedWallStop` output sources remain compiled.

## Scope

- Delete the unreachable direct stopped-vehicle controller, private latch/phase state and resets.
- Delete direct-only retained-pass/rejoin/wall-stop helpers and tests.
- Delete `LowSpeedDirect` and `LowSpeedWallStop` final-source representations.
- Delete the `LowSpeedDirect` execution-contract formulation.
- Delete direct-only configuration fields and YAML keys.
- Remove the obsolete direct-owner input from stopped-vehicle line ownership.

## Explicit non-scope

- No clearance, weight, solver, horizon, rate, timeout or behavior tuning.
- Do not delete stopped-vehicle detection, side selection, local corridor construction or static-wall
  preflight used by canonical MPCC.
- Preserve the live low-speed local-path shift-speed reference.
- Preserve path feedback used by bounded solver-failure crawl.
- Do not change canonical Rejoin intent or external Emergency/Recovery supervision.

## Acceptance

- A failure-first source contract proves all direct-owner representations are physically absent.
- Focused source contracts pass.
- `make autoware-build` passes.
- Package tests pass with `BUILD_TESTING=ON`.
- No user-owned generated result is staged.

## Rollback

Revert the single commit produced by this Slice.
