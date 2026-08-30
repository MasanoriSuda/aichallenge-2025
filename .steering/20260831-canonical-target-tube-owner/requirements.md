# Requirements: canonical target-tube owner

## Objective

Classify and repair the first active ShiftOut failure after the asynchronous
Gate A bridge without adding a lease, grace period, fallback, solver change or
clearance change.

Frozen evidence:

- baseline: `a8b8cbe3`
- run: `output/20260831-060156`
- domain: `d2`
- first causal failure snapshot:
  `000000000864-09b0c6a53f36542f-shiftout-side-negative-dynamic-obstacle-refinement-solve-rejected`
- decision: `1483`
- source sequence: `864`
- immutable interaction fingerprint: `698276355274855471`

## Expected behavior

Every active Overtake solve consumes the target tube produced from the current
control epoch and sealed into the canonical seven-state submission. Stateless
candidate generation may replace Mission path geometry, but it must not run a
second target predictor with a different coordinate window.

## Observed behavior

The canonical submission already contains a stage-wise target tube. The
stateless current-world candidate discards it and reconstructs another tube by
projecting constant global velocity onto the ego solver's finite wall window.
At decision 1483 the target reaches that window's terminal progress, remains
pinned there for all 20 stages, and its residual is reinterpreted as lateral
motion from `-1.170 m` to `-8.143 m`.

At stage 10 this produces mutually exclusive hard constraints:

- wall corridor: `e_y >= -3.836 m`
- selected-side dynamic row: `e_y <= -6.236 m`

The QP cannot satisfy both. Solver rejection, retained-proof loss, Emergency
Stop and finally `locked target stale or lost` are downstream consequences.

## Scope

- Make the current-epoch canonical target tube the only target-prediction
  owner used by stateless candidate generation.
- Delete the duplicate finite-window world projection from
  `mpcc_stateless_maneuver`.
- Keep Mission path/reference geometry excluded from stateless candidates.
- Preserve the exact ReplayWorld dynamic proof as final physical authority.
- Add deterministic tests for the target-tube ownership invariant.

## Non-scope

- no production authority change;
- no Mission resume/retry/timeout/lease/grace rule;
- no OSQP tolerance, iteration or weight change;
- no wall, vehicle or dynamic clearance change;
- no target-continuity extension;
- no parameter tuning.

## Definition of Done

- A failure-first test proves a sealed canonical tube cannot be replaced by a
  projection clamped to the wall-window endpoint.
- Missing or identity-incompatible canonical tubes fail closed.
- The duplicate course projection implementation and its tests are deleted.
- Build and full package tests pass.
- A dynamic run no longer creates a target tube pinned to the ego wall-window
  endpoint before the previous decision-1483 failure boundary.
- Any next failure is frozen and classified separately.
