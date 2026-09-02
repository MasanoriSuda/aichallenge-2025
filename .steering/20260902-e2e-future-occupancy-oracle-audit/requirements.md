# Requirements

## Objective

Test whether time-indexed obstacle occupancy, absent from the current E2E
teacher, separates the frozen failed peer encounter from successful teacher
runs.

## Constraints

- Offline privileged oracle only; future scans may never become runtime input.
- Reuse the exact candidate bank and physical footprint contract from the
  current-scan audit.
- Use causal pose samples to transform each future scan into the current
  base-link frame.
- Do not claim visibility for geometry absent from a future scan.
- Compare successful and failed bags with frozen parameters.
- Do not generate labels or change production until discrimination is proven.

## Definition of Done

- Time-aligned future point clouds can certify a full candidate trajectory.
- Synthetic tests cover coordinate transforms and moving-obstacle separation.
- Successful and failed runs are replayed with the same sample budget.
- The oracle is accepted or rejected before training/runtime changes.
