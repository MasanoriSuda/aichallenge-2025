# Design

## Candidate source resolution

A pure resolver in `v2x_overtake_core` selects one candidate per side in this
order:

1. feasible non-progressive `selected_mission`;
2. feasible `mpcc_receding_mission`;
3. feasible progressive `selected_mission`;
4. unavailable.

The candidate must match the requested side. The resolver returns provenance
and whether the selected candidate is prefix-only.

## Branch solve

The existing isolated left/right extended-MPCC snapshots consume the resolved
candidate. A progressive candidate is frozen only inside its isolated branch,
and retains all bounded horizon metadata and `progressive_entry=true`.
Production problem construction therefore remains responsible for stage wall,
target, speed and physical trajectory validation.

## Atomic live handoff

After both solves finish, the chosen resolved candidate is copied atomically
with its side assessment. Entry and active replacement use the existing
progressive admission gates. The bridge does not convert a prefix to a complete
Mission and does not bypass no-return or hard-fault checks.

## Diagnostics

The one-second worker status reports candidate source, attempted/feasible,
objective, bound reserve and failure reason for both sides. This distinguishes
"no candidate reached MPCC" from a real extended-QP failure.
