# Design

## Existing dataflow

```text
actual_wall_monitor_pose_ -------------------------------> swept path start
model->spatial_state --------> five-state x0 hard equality
five-state QP + u0 ----------> solved stage-1 state
solved theta ----------------> course-frame world pose
current pose + solved pose ---> linear world-pose interpolation certificate
```

The two start-state paths are not currently compared, and the certificate does not retain the first
input or stage integration period. Therefore path-index-1 cannot distinguish projection mismatch,
true dynamic unreachability and interpolation artefact.

The five-state state contract additionally contained an unpaired omission:

```text
predicted world pose --projection--> [e_y, e_lag, e_psi]
                                      e_lag was forced to 0

solved [e_y, e_lag, e_psi, theta] --world reconstruction-->
                                      e_lag was discarded
```

Those two errors could cancel numerically, so changing only the certificate or only x0 would be an
unsafe partial fix.

## Diagnostic contract

Introduce a typed first-stage reachability record containing:

- measured world pose;
- state-zero world pose reconstructed from solved-progress course-frame provenance;
- solved first-stage world pose;
- current speed, solved acceleration/curvature/virtual progress speed and stage period;
- constant-curvature integrated endpoint;
- measured-to-state-zero and integrated-to-solved position/yaw residuals;
- conservative endpoint interpolation wall result;
- control-derived rollout wall result.

The diagnostic is emitted only when the physical certificate rejects the current-to-first-stage
connector, and is aggregated by classification. It does not change certificate acceptance in the
measurement step.

## Candidate fixes after measurement

### If H1 is confirmed

Make a control-derived first-stage swept rollout part of the canonical physical certificate and
reject a QP endpoint whose integrated endpoint residual exceeds the accepted model/projection
tolerance. The optimizer may later need a reachable-set or shorter first stage, but no post-solve
clamp is added.

### If H2 is confirmed

Replace only the first connector's linear interpolation by the command-defined, densely sampled
bicycle rollout. Later stage-to-stage segments keep their existing conservative proof until their
inputs are similarly preserved and tested.

### If H3 is confirmed

Unify state-zero construction around the measured world pose and canonical course-frame projection.
Do not compensate downstream with extra wall margin.

## Selected structural correction

1. Define a typed, invertible world-pose/Frenet-pose contract carrying `e_y`, `e_lag` and `e_psi`.
2. In Track/Cruise shadow only, construct five-state x0 from the delay-compensated control pose and
   the exact progress-origin waypoint, including signed `e_lag`.
3. Preserve solved `e_lag` through extraction and reconstruct certificate poses from the complete
   state. Production overtake authority remains on its existing contract in this Slice.
4. Keep the conservative physical certificate. Add separate raw-to-predicted and predicted-to-stage
   wall rollouts solely as decision-scoped failure provenance.

The remaining H1 events cannot be repaired by loosening the candidate wall proof. They show that the
legacy production controller has already consumed the recoverable prefix before a canonical
Track/Cruise solution can be adopted. Slice 3 must therefore start canonical authority from a fresh
certified state and use a bounded last-certified canonical solution; it must not attempt a late
cycle-local handoff after the predicted pose is already inside the wall footprint.

## Safety discipline

Until measurement selects one case, the existing conservative certificate remains the acceptance
oracle. A diagnostic alternate rollout must never make a rejected shadow solution selectable.
