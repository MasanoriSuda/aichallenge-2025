# Design

## Before

```text
deleted synchronous extended solver producer
  X-> record failure/success

compiled migration state
  -> circuit active() reads (always false)
  -> reentry streak telemetry (always zero)
  -> mode handoff reset() calls (never resolves)
  -> DP authority configuration for an impossible degraded state
```

## After

```text
canonical production owners
  -> same-formulation solve/certificate/admission
  -> DP authority receives no nonexistent cross-formulation degradation input
  -> explicit Emergency on missing canonical proof
```

The change removes an impossible state rather than replacing it with a boolean
alias or compatibility flag. Current extended five-state solvers used by
Follow/Overtake/Rejoin and six-state Track/Cruise remain intact; only the old
MPC/MPCC switching mechanism disappears.

## Deletion accounting

- Production branches added: 0.
- Runtime configuration added: 0.
- Migration branches removed: circuit degradation, requalification and
  cross-formulation velocity blending.
- Configuration removed: extended reentry/handoff and DP block-on-retired-
  circuit keys.
- Remaining legacy boundary: retained DynamicEscape/wall-handoff arbitration
  in the final publisher, to be audited separately.

## Failure behavior

Canonical solve/certificate failures continue to produce the existing typed
Emergency result. They cannot transfer to another normal formulation. This
Slice does not alter Recovery behavior.
