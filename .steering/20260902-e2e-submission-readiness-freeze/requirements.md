# Requirements

## Objective

Freeze the current E2E submission evidence after the temporal and privileged
teacher audits, without changing production authority or tuning parameters.

## Constraints

- Preserve the qualified raw and spatial artifact identities.
- Keep single-vehicle qualification separate from multi-vehicle readiness.
- A failed peer Gate may not be hidden by a successful single-vehicle Gate.
- An inconclusive privileged oracle may not authorize labels or runtime input.
- Do not modify participant launch defaults, checkpoints or controller code.
- Do not commit generated reports, bags or result JSON.

## Definition of Done

- One deterministic tool verifies artifact identity and the frozen Gate files.
- The report distinguishes reject, single-only and multi-vehicle-ready states.
- Tests cover identity mismatch and the single-only decision.
- The frozen repository is documented as single-vehicle qualified and
  multi-vehicle rejected.
