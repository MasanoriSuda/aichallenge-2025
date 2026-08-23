# Follow warm-start dynamic evidence

## Purpose

Verify that the typed trailing dual layout restores live Follow warm starts and improves QP/canonical
availability without changing production authority.

## Baseline

The preceding run recorded 3110 valid-contract attempts, 2573 canonical-ready results (82.73%), zero
warm starts, and 537 solve-boundary losses.

## Acceptance

- `Follow MPCC shadow runtime` reports nonzero warm starts.
- Aggregate valid-contract solve/canonical rate is measured against 82.73%.
- Physical-gap, wall, and canonical boundary regressions are reported separately.
- All Follow results remain `authority=shadow, selected=0`.
- The user-owned result JSON remains unchanged.
