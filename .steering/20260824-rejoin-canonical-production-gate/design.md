# Design

## Existing authority break

```text
OvertakeLine Recovery
-> canonical intent Rejoin
-> isolated five-state Rejoin shadow solve
-> shadow result is discarded
-> legacy three-state normal solve publishes
```

The trigger may be a wall or execution failure, but the formulation switch is
deterministic in source and is therefore a separate architectural defect.

## Qualification sequence

1. Reproduce Rejoin on the unmodified baseline and record fresh solve,
   physical certificate, canonical-chain and callback timing coverage.
2. Classify fresh misses by producer: build, solve, physical current pose,
   physical future path, or artifact/authority construction.
   Current-HEAD evidence classifies the blocker as future physical wall
   rejection followed by current-pose wall contact, despite successful QP
   solves.
3. Join the fixed five-state initial condition to the first affine dynamics
   block.  State zero is the delay-compensated execution pose, so stage zero
   must be linearized at that same lateral/lag/heading/velocity state and at
   the currently reachable curvature.  Horizon timing remains owned by the
   immutable stage-geometry schedule.
4. Decide retained policy from evidence:
   - Reusing Track/Cruise empty-world proof is allowed only when the current
     obstacle observation is explicitly empty.
   - A current target or other active vehicle requires a current-world dynamic
     obstacle proof; an old plan or age cannot stand in for it.
   - Until such proof exists, a fresh miss must select explicit Emergency, not
     a legacy normal controller.
5. Add failure-first authority/deletion tests.
6. Promote only the already-certified canonical selection and delete the
   Rejoin legacy fallthrough in the same change.
7. Repeat the dynamic Gate.

## Rejected approaches

- Classify Rejoin as Track/Cruise solely to inherit its retained plan.
- Accept a Rejoin plan because it is recent.
- Relax wall contact or OSQP parameters to improve coverage.
- Keep legacy three-state as a hidden recovery fallback after promotion.
- Generate a progress-indexed physical wall profile synchronously for every
  Track/Cruise/Follow/Rejoin cycle.  Dynamic Gate
  `output/20260824-183322` showed that this did not remove the first-stage
  exact-vs-affine mismatch and raised normal callback averages to roughly
  29--37 ms with sustained overruns.

## Accepted root-cause repair under qualification

The first Rejoin failure in `output/20260824-183322` had a clear exact
measured-to-control and first-command rollout, while the QP-reconstructed
stage zero contacted the wall.  The lag-aware QP endpoint was 0.408 m from
the exact constant-curvature endpoint.  A later Track/Cruise sample exposed
the same class with 0.376 m position and 0.092 rad yaw mismatch.

The builder fixed state zero from the delay-compensated execution pose but
linearized its outgoing transition at the desired path and desired velocity.
That makes one equality row describe two different operating points.  The
repair anchors only the first transition at the fixed execution state,
measured velocity and previous reachable curvature; later stages remain
nominal trajectory linearizations.  Stage duration is now passed explicitly
from the existing immutable schedule so changing the anchor cannot change the
horizon clock.

## Production boundary

Qualified Rejoin returns at the canonical boundary before the legacy normal
solver. A complete fresh artifact publishes through `canonical_normal_control`.
Every incomplete build, solve, physical certificate or artifact chain selects
`canonical_normal_emergency_stop`; no retained or legacy normal alternative is
introduced. Telemetry is labelled as production and reports whether that exact
cycle selected the canonical artifact.
