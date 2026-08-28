# IM-3 same-snapshot A/B proof design

## Data flow

```text
RecordedInteractionSnapshot (one immutable fingerprint)
  +-- A: recorded persistent candidate Snapshot
  +-- B-left: stateless rebuild(-1)
  `-- B-right: stateless rebuild(+1)
          |
          v
     independent SolverContext per arm
          |
          v
     exact nonlinear trajectory
          |
          +-- physical wall sweep
          +-- timed dynamic-opponent sweep
          `-- terminal successor viability
                    |
                    v
             ManeuverBundle (data only)
```

## Evidence closure

`ReplayWorld::control_prefix_elapsed_sec` is the exact timing already owned by
`CanonicalCurrentControlPath`.  It starts at zero and ends at
`control_prediction_origin_sec - observed_sec`.

`ReplayWorld::physical_footprint` is copied from the physical wall snapshot
before hard wall clearance is added.  The existing
`Snapshot::wall_footprint` remains the expanded refinement footprint.  Keeping
both meanings explicit prevents a second clearance expansion during replay.

## Comparison isolation

The A and B arms use separate `SolverContext` objects so one arm's solved
iterate cannot seed another.  Both use the compiled-in production SQP policy;
IM-3 introduces no alternate setting.  Arm identity is data provenance, not
runtime authority.

## Certificates

The wall certificate is reconstructed from the exact artifact trajectory and
the replayed physical wall inputs.  The opponent certificate sweeps the raw
ego footprint over the timed control prefix and exact trajectory against all
same-generation obstacle circles.

The opponent motion model remains the production constant-velocity circle
model.  Acceleration and covariance stay in the immutable source; covariance
has already contributed to the resolved radius.  Changing the prediction
model would confound lifecycle comparison and is not part of this Slice.

## Bundle boundary

The bundle contains target/homotopy identity, exact trajectory, wall and
opponent certificates, terminal successor, and both source/candidate
fingerprints.  It has no command conversion and cannot enter the certified
production store.
