# Design

## Cause and propagation

1. The broad seven-state problem solves with union lateral support.
2. Physical wall refinement samples a local progress/pose bucket around that
   solved trajectory.
3. Production installs bucket lateral/progress state boxes and swept affine
   rows as hard constraints.
4. At the frozen stage 1, the bucket requires a lateral state outside the
   dynamics/input reachable interval, so the refined QP is empty.
5. Normal Cruise authority disappears although replaying either control
   sequence through the canonical nonlinear model is exact-wall clear.

## Change

Production keeps:

- broad semantic/union state support;
- progress-aligned planning corridor rows;
- canonical nonlinear reconstruction;
- exact occupancy-grid wall proof;
- timed dynamic-obstacle and recursive terminal Stop proof.

Production no longer installs local physical pose-bucket lateral/progress
state boxes or physical swept affine rows as execution-authoritative hard QP
constraints.  The bucket resolver and cache remain available as diagnostics.
Observation-only wall-bucket audit modes continue to install the historical
hard rows so the old and new ownership can be compared on frozen snapshots.

This removes duplicate wall ownership.  It does not weaken the exact physical
acceptance gate: an unsafe candidate still has no Bundle and no publisher
edge.

## Verification

- Frozen decision 2177: production persistent arm changed from an infeasible
  refinement QP to an accepted certified Bundle (`progress=12.9628 m`,
  `velocity=7.99676 m/s`, `lateral_reserve=0.305762 m`).
- Full build: 25 packages passed.
- Package tests: 59/59 passed.
- Dynamic run: `output/20260831-051051`, `make dev3`.
  - `bucket_hard=0`: d1=75, d2=69, d3=48; `bucket_hard=1`: zero.
  - Historical `rate-resolved wall refinement rejected:`: zero on all cars.
  - Actual-footprint wall-margin violation: zero on all cars.
  - Exact proof still rejected physically occupied candidates on d2, so the
    fail-closed wall certificate remained active.
  - Canonical production selections: d1=141, d2=159, d3=127; ShiftOut intent
    was exercised on d1 and d2.

Follow/SafetyBrake transitions, candidate-generation latency and completion
through Pass/Return remain separate failure families.  They are not grounds
for changing this Slice's wall proof ownership.
