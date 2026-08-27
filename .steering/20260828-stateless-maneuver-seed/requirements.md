# IM-2 stateless receding maneuver seed requirements

## Baseline

- Branch: `develop_july`
- Rollback commit: `fa063748`
- Production authority, solver policy, configuration and command publication
  remain frozen.

## Repaired invariant

For one replay-ready Interaction Snapshot, either pass homotopy must be
constructible without reading:

- persistent Mission path or age;
- ShiftOut/Pass/Return phase state;
- retained candidate availability;
- previous tactical path, lease, timeout or retry state.

The result is a candidate input for the existing seven-state SQP.  It is not
an executable command and cannot be promoted by this Slice.

## Earliest current violation

The previous A/B experiment could expose the active persistent Mission but its
"fresh receding" arm was merely the optional product of that same Mission
lifecycle.  During active ShiftOut it was absent, so comparison stopped before
problem construction.

## Scope

- Define a stateless maneuver seed and explicit rejection reasons.
- Rebuild left or right lateral/heading references from the immutable wall and
  target stage data in the Interaction Snapshot.
- Preserve the same velocity/progress horizon, weights, actuation limits,
  physical wall inputs and target tube used by A.
- Seal the rebuilt candidate with a deterministic fingerprint.
- Describe a non-authoritative terminal successor intent: Return when visible,
  otherwise a Stop suffix when the semantic bounds permit braking.
- Provide an offline CLI and focused tests.

## Non-scope

- No production worker or controller hook.
- No new Mission, phase, lease, grace, timeout, retry or fallback.
- No SQP execution or physical certification; IM-3 uses the unchanged solver
  and proof pipeline.
- No spline/lattice C arm and no nonlinear D arm.
- No parameter or clearance change.

## Files

- `include/.../mpcc_stateless_maneuver.hpp`
- `src/mpcc_stateless_maneuver.cpp`
- `src/mpcc_maneuver_replay.cpp`
- `test/test_mpcc_stateless_maneuver.cpp`
- `CMakeLists.txt`

## Definition of done

- The same complete snapshot independently produces both side seeds.
- Changing only persistent-Mission lateral/heading references does not change
  the rebuilt stateless path.
- World, target, side and rebuilt request provenance are sealed.
- Incomplete/mixed-generation input and an unavailable terminal successor are
  rejected explicitly.
- The type has no publisher, mailbox, certified-plan store or command API.
- Focused tests, package tests and `make autoware-build` pass.
