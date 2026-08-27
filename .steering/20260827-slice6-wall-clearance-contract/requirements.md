# Requirements

## Objective

Preserve the two distinct wall-clearance meanings already defined by
`WallClearanceContract` through the canonical seven-state MPCC pipeline:

- planning/admission uses `required_clearance_m`;
- QP physical refinement, fresh proof, and retained proof use
  `physical_clearance_m`.

The values must share one immutable producer, but must not be collapsed into
one number.

## Evidence boundary

- Baseline: `86b2028`
- Run: `output/20260827-175828`, Domain 1
- Last normal authority: ShiftOut before decision 6762
- First abnormal authority decision: 6762

The problem advertised a required wall clearance of 0.40 m, while the
immutable physical snapshot stored 0.0 m.  QP physical refinement consumed
the unexpanded footprint.  The accepted trajectory therefore had no explicit
physical-clearance evidence.

The first implementation hypothesis reused the 0.40 m required planning
clearance for physical proof.  Run `output/20260827-182448` refuted that
hypothesis: Track/Cruise became over-constrained and the wall-refined QP
repeatedly reached maximum iterations.  The existing contract intentionally
separates physical clearance from planning clearance plus runtime reserve.

## Constraints

- Do not tune wall margins, solver settings, horizons, timeouts, or costs.
- Do not add fallback, retry, grace, or a second normal authority.
- Do not relax occupied, unknown, or out-of-map rejection.
- Preserve ROS 2 and evaluation interfaces.

## Definition of Done

- Every canonical problem owns both the physical and required planning
  clearance before Mission admission.
- Current/future physical envelopes, QP physical refinement, fresh proof, and
  retained proof consume the same physical clearance exactly once.
- Overtake planning/admission continues to consume the required clearance.
- Invalid or missing physical clearance fails before a canonical physical
  snapshot is produced.
- Focused and full package tests pass.
- `make dev2` demonstrates that Track/Cruise remains feasible and ShiftOut
  pre-entry can obtain a physical wall certificate without a missing-contract
  rejection.
