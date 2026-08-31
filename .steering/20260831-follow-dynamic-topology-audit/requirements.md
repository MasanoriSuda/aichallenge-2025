# Requirements

## Objective

Classify the frozen decision 1187 Follow authority loss without changing
production authority or controller parameters.  Determine whether the
physically certified warm-start controls are rejected because Follow uses the
`Automatic` dynamic-obstacle topology instead of an explicit longitudinal
`StayBehind` topology, or because the affine seven-state model cannot represent
the same nonlinear trajectory.

## Frozen evidence

- Run: `output/20260831-192221`, Domain 1
- Decision: `1187`
- Interaction fingerprint: `13608911548693048044`
- Requested intent: `Follow`
- Recorded topology: `Automatic`, pass side `0`
- Production symptom: no current-world normal authority followed by external
  Emergency Stop
- Existing architecture result: A/B/C/D do not produce a certified QP bundle
- Independent physical oracle: the recorded warm-start controls produce an
  exact wall/dynamic/terminal-certified trajectory

## Constraints

- Do not change production authority, runtime parameters, solver tolerances,
  clearance, timeout, lease, grace, retry, or fallback behavior.
- Do not classify local solver failure as physical infeasibility.
- Every comparison must use the same immutable world, geometry, prediction,
  dynamics, cost, horizon, and terminal successor.
- A changed topology must receive a new deterministic candidate fingerprint.

## Acceptance

- The audit reports `Automatic` and `StayBehind` results side by side.
- Focused tests prove the audit has no publisher/store/command authority.
- The frozen snapshot is replayed and the earliest divergent constraint or
  model boundary is identified.
