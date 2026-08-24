# Audit

## Confirmed source state

- `ControlIntent::Rejoin` is accepted by the canonical intent contract.
- Rejoin owns an isolated five-state solver context, plan store, warm-start
  identity and telemetry path.
- Retained Rejoin is intentionally disabled.
- `get_control()` evaluates Rejoin shadow but does not return its selection or
  Emergency; execution continues into the legacy three-state normal path.

## Historical evidence

Earlier 2026-08-24 runs exercised Rejoin shadow. Some windows produced complete
fresh canonical chains, while others were dominated by current/future hard wall
contact and first-curvature convergence failures. Those runs predate the
current stage-lineage and duplicate first-curvature-owner repairs and cannot by
themselves authorize production promotion.

## Current-HEAD dynamic evidence

Run: `output/20260824-181825/d1/autoware.log`

The current HEAD reached Rejoin at decision 2485 after
`FollowPrepare -> Recovery`. The isolated five-state Rejoin producer was not
missing and was not dominated by solver failures:

- Rejoin shadow cycles observed: 38
- QP solves: 38 / 38
- physically certified fresh plans: 0 / 38
- future solution hard-wall contact: 10
- current-pose hard-wall contact after entry: 28
- solve time average / maximum: 5.577 / 19.991 ms
- total producer time average / maximum: 5.829 / 20.976 ms

At the first Rejoin cycle the five-state QP solved in 2925 iterations, but its
first solved state was rejected as a hard wall contact:

```text
stage=0, wp=178
lateral=2.059 m, lag=0.069 m, heading_offset=0.357 rad
scalar bounds=[-4.460, 3.397] m, scalar reserve=1.338 m
physical certificate=hard-wall-contact
```

The same decision then fell through to
`authority=legacy-normal-bypass`, `formulation=legacy-spatial-mpc-3state`, and
published the legacy normal command. This proves both halves of the break:

1. Rejoin still has a legacy production authority.
2. Removing it now would select Emergency for the full observed Rejoin window,
   because the canonical producer has no physically certified result.

## Falsified wall-profile-only hypothesis

A candidate change generated a physical progress wall profile for every
canonical normal intent.  Dynamic run `output/20260824-183322` proved that the
profile could be coupled, but it did not remove the Rejoin stage-zero physical
rejection and introduced sustained callback overruns.  The candidate was
removed rather than retained as another fallback or feature flag.

## Root cause

The QP state zero is fixed at the delay-compensated execution pose.  Its first
affine transition, however, was linearized at the desired path lateral,
heading and velocity rather than that fixed state.  The final footprint proof
then compared the resulting approximate stage-zero endpoint against the exact
world map.

At decision 4036 the actual command rollout was wall-clear, but the lag-aware
QP endpoint was 0.408 m from its exact constant-curvature endpoint and was
reported as a wall contact.  Track/Cruise later showed the same class with a
0.376 m position and 0.092 rad yaw discrepancy.  This is an upstream dynamics
join defect; weakening the physical certificate would hide it.

The accepted repair under test makes the first transition tangent at the
fixed initial Frenet state, measured speed and reachable previous curvature.
It also separates immutable stage timing from the selected linearization
velocity.  Later stages and all safety margins remain unchanged.

## Qualification after root-cause repair

Run: `output/20260824-191213/d1/autoware.log`

- First Rejoin cycle: maximum iterations, no canonical selection.
- Following window: 33 / 33 solved, physically certified and complete through
  extraction/store/authority/actuation.
- Recovery exited once.

This proves the producer after the first-transition repair. Retained Rejoin is
still not qualified; a fresh miss must therefore be an explicit Emergency.

## Post-promotion dynamic Gate

Run: `output/20260824-192226/d1/autoware.log`

- Rejoin eligible/build/solve: 83 / 83 / 83.
- Fresh physical/canonical selections: 69 / 83.
- Exact wall certificate rejects: 14 (13 hard contact, 1 swept path).
- Sampled final Rejoin decisions: 12 fresh canonical, 9 explicit Emergency.
- Sampled Rejoin legacy-normal decisions: 0.
- Canonical contract join failures: 0.
- `Recovery -> FollowPrepare` completion: 1.

The authority Gate passes: every observed normal Rejoin decision is either a
matching five-state certified command or an explicit Emergency override. The
14 wall rejects and callback overruns around the Recovery window are a runtime
quality concern, not permission to restore the deleted three-state authority
or weaken the physical certificate. The run also exposed a telemetry defect:
the aggregate still said `authority=shadow` after promotion. The telemetry was
renamed to canonical production and its selected bit now reflects the same
artifact used at the final authority boundary.
