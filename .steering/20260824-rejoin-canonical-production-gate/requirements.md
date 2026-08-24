# Requirements

## Objective

Close the remaining normal-authority switch at `ControlIntent::Rejoin` without
promoting unqualified evidence or restoring another fallback path.

## Baseline

- Source baseline: `b0a46a4`.
- `ShiftOut`, `Pass`, and `Return` already return through the canonical
  five-state production boundary.
- `Rejoin` is supported by the canonical contract and has an isolated solver,
  plan store and warm-start identity, but is still observation-only.
- The current `get_control()` deliberately falls through from Rejoin shadow to
  the legacy three-state normal controller.

## Invariants

- Do not promote Rejoin before current-HEAD dynamic evidence exercises it.
- Do not reuse Track/Cruise solver or warm-start state.
- Do not treat an old wall certificate or age alone as retained authority.
- Do not add timeout, lease, retry, fallback, feature flag or parameter tuning.
- Promotion and deletion of the Rejoin legacy normal branch must be atomic.
- Emergency and Stuck/gear/reverse Recovery remain independent authorities.
- Preserve `aichallenge/result-summary.json`.

## Definition of done

- Current-HEAD Rejoin fresh-chain coverage is classified dynamically.
- Fresh, retained and Emergency ownership rules are explicit and tested.
- If evidence is insufficient, production remains unchanged and the exact
  blocker is recorded.
- If evidence is sufficient, Rejoin cannot reach the legacy normal solver.
- Focused/package tests, build and a post-change dynamic Gate pass.
- The Slice is committed without generated run artifacts.
