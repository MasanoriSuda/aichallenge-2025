# Overtake canonical fresh-chain shadow

## Purpose

Prove that an already solved five-state Overtake/Dynamic Escape primal can become one exact,
physically certified canonical execution plan and command without conversion to the three-state
execution layout.

## Root-cause evidence

The accepted low-speed replay removed the obsolete direct owner but exposed 31 normal-formulation
transitions in the bounded replay. The live Overtake path currently does this:

```text
five-state result
-> convert_extended_solution_to_legacy()
-> legacy prediction/post-processing/publication

five-state unavailable or requalifying
-> solve three-state progress problem in the same cycle
-> publish a different normal formulation
```

The final trace therefore reports `canonical=violated` even for a physically checked five-state
solution because no immutable plan/cursor/canonical-command identity reaches publication.

## Scope

- Consume the existing live five-state solve result; do not add a second solve.
- Normalize the exact primal using its row residual/tolerance contract.
- Extract and physically certify the exact five-state execution trajectory.
- Build the immutable canonical plan, cursor, fresh authority and command.
- Compare the canonical actuation with the direct actuation from the same normalized primal.
- Emit aggregated Gate A telemetry while leaving production output unchanged.

## Non-scope

- Do not promote canonical Overtake authority in this Slice.
- Do not delete the three-state fallback until fresh coverage and retained-plan design are proven.
- Do not tune OSQP, weights, horizon, wall clearance, timeout, lease or cooldown parameters.
- Do not change Mission, target selection, branch selection or path geometry.

## Acceptance

- Static tests cover eligible and ineligible intent classification.
- Build and package tests pass.
- Deterministic replay observes ShiftOut/Pass/Return fresh-chain attempts.
- Every accepted fresh chain has complete problem/solution/plan/cursor/command identity.
- Canonical and direct first actuation differ by no more than `1e-12`.
- No canonical shadow result affects final output.
