# Requirements

## Objective

Close the next rate-resolved Track/Cruise production precondition: prove that
one complete six-state execution artifact is physically wall-clear from the
current measured pose through the solved horizon.

## Root-cause boundary

The accepted six-state artifact currently proves QP rows, state/input shape,
steering-state dynamics and publication sampling. It does not prove the kart
footprint against the static wall map. QP lateral boxes are not a substitute
for a swept-footprint certificate.

## Scope

- Retain the solve-time course-progress origin and nominal stage distances in
  the immutable artifact.
- Convert states 1..N into the existing exact physical trajectory contract.
- On the control thread, require current intent and stage-geometry identity to
  match before using the current reference path and wall map.
- Reuse the established swept-footprint wall checker from the current pose.
- Keep the result observation-only and preserve typed reject provenance.
- Add deterministic unit and source-contract tests.

## Non-goals

- No production authority or publisher connection.
- No parameter, wall-margin, cadence, fallback or solver tuning.
- No retained-artifact admission.
- No Follow/Overtake obstacle-tube proof; the rate-resolved producer remains
  Track/Cruise-only in this Slice.
- Do not edit or commit `aichallenge/result-summary.json`.

## Definition of Done

- Every current-semantic solved artifact is either accepted by the existing
  physical wall sweep or rejected with exact adapter/wall provenance.
- Source identity mismatch is measured separately and never treated as a wall
  rejection.
- Runtime remains `authority=shadow, selected=0`.
