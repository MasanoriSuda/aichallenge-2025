# Slice 2 design

## Causal statement

The current controller couples progress metadata construction to live progress-formulation
activation. Because Track/Cruise is intentionally excluded by the overtake-only migration boundary,
the canonical five-state builder has no valid input during ordinary racing. This prevents proving
that the target formulation is feasible and timely before authority promotion.

Slice 2 separates two concepts:

- `progress_contouring_active`: the formulation that currently owns production execution;
- `progress_metadata_available`: a complete, immutable input representation from which the
  canonical five-state shadow problem can be built.

Only the second becomes true for Track/Cruise. The first remains false.

## Data flow

```text
production observation + stage geometry + wall bounds + references
                         |
            +------------+-------------+
            |                          |
            v                          v
 legacy Track/Cruise problem     progress metadata copy
            |                          |
            v                          v
 production legacy solve       five-state shadow build/solve
            |                          |
            |                    swept wall certificate
            |                          |
            +------------+-------------+
                         v
          difference/timing/coverage telemetry only
                         |
                         v
           unchanged production command publisher
```

## Shadow eligibility

Use the existing migration boundary instead of adding a flag:

```text
progress_contouring_mpcc_enabled
&& progress_contouring_mpcc_overtake_only
&& extended_dynamics_enabled
&& live intent in {Track, Cruise}
&& live progress formulation inactive
&& ordinary control-cycle problem (not a tactical snapshot)
```

This condition has a deletion gate: it disappears when Slice 3 promotes Track/Cruise and the
overtake-only boundary no longer excludes those intents.

## Warm-start contract

The shadow solver has a dedicated persistent context. A shifted warm start is compatible only when:

- intent and five-state formulation are unchanged;
- horizon and state/input/bounds/cost schemas are unchanged;
- stage geometry is identical or a forward rolling overlap of the preceding geometry;
- progress origin remains continuous.

Intent/schema/horizon/non-overlapping geometry changes increment the context epoch before solve.
Progress discontinuity is also checked inside the existing five-state solver and cold-resets its
workspace. Shadow state is never shared with live or tactical branch solvers.

## Physical certificate

The converted five-state prediction is checked with the same stage geometry and lateral bounds used
to construct the problem. The current physical footprint is swept to every predicted stage against
the occupancy grid. The shadow certificate is diagnostic only and cannot authorize a command.

## Telemetry

Aggregate once per second and emit outcome transitions immediately:

- eligible / metadata-valid / attempted / solved / finite / constraint-valid / certified counts;
- build, solve and certificate failure reason;
- solve mean/max, iterations mean/max, warm/reset counts;
- absolute first speed and steering difference from production;
- RMS/max lateral prediction difference and terminal progress/velocity;
- context/geometry identity and explicit `authority=shadow`.

## Files

- Update `race_mpcc_foundation.hpp/.cpp` with pure eligibility and rolling warm-start contracts.
- Update `test_race_mpcc_foundation.cpp` with deterministic contract tests first.
- Update `mpc_controller_cpp.cpp` to separate metadata availability, run the dedicated shadow solve,
  certify it and log aggregate telemetry.
- Update `docs/spec/mpc-integration.md` only after static and dynamic validation establishes the
  implemented current behavior.

## Branch/configuration accounting

- New production command branches: 0.
- New runtime configuration: 0.
- Deleted production command branches: 0 (shadow-only exception approved by Slice 2).
- Remaining normal authorities: legacy Track/Cruise, three-state progress, five-state overtake,
  direct/derived bypass paths. Their deletion milestones remain Slices 3-6.
