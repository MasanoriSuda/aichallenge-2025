# Design

## Evidence before implementation

`output/20260826-150956/d1/autoware.log` entered ShiftOut through an exact
six-state Gate A.  The six-state worker then reported 71 consecutive solved
results.  The first causal failure was not OSQP:

```text
retained=velocity-unreachable
current=3.978619
expected=4.179448
reachable_upper=4.176722
duration=0.130000
```

That rejection immediately selected Emergency braking.  Only after the state
was disturbed did the QP reach maximum iterations and retained plans become
stale.

## Hypotheses and result

1. **Falsified:** the current speed belongs to an older odometry source
   timestamp.  The MCAP shows the rejection used the odometry sample whose
   source timestamp exactly equals the retained request time (`24.694999 s`).
2. **Not causal:** the retained cursor is internally consistent with the
   certified plan time contract.
3. **Observed downstream:** the retained velocity is physically outside the
   acceleration-reachable interval, but this happens after execution geometry
   diverges from the Gate A proof.
4. **Confirmed structural cause:** the causal Gate A proposal stores its
   `CertifiedPlan`, but copies the un-certified tactical Mission into the FSM.
   `freeze_selected_overtake_mission()` then enables and later prefers the old
   tactical `frenet_dp_path_*` vectors.  The six-state physical snapshot is
   discarded at the boundary.  The first active problem is therefore allowed
   to use a geometry source different from the one Gate A certified.

The moving log proves the mismatch directly: admission prints
`certificate=0` while `execution_samples=31`, even though the Gate A worker
reported `physical=accepted`.  The 31 samples are the tactical DP path; the
accepted six-state horizon has 20 stages.

## Repair

Bind the `CertifiedPlan::physical_snapshot->trajectory`, course-progress
origin, wall clearance and target provenance into the Mission before Gate A is
published.  During freeze, an exact physical certificate has precedence over
the tactical DP path.  A later path may replace it only after the existing
six-state solved/current-world/physical promotion contract succeeds.

## Prohibited symptomatic repairs

- enlarge `physical_global_tolerance`;
- relax the comparison with a magic epsilon;
- retain the plan after `VelocityUnreachable` unconditionally;
- increase solver iterations after the downstream collapse.
