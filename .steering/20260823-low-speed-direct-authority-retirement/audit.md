# Root-cause audit

## Observed phenomenon

At decision 1944 the controller detected stopped vehicle `d2` at 6.30 m and selected a feasible
left Dynamic Escape path. `LowSpeedDirect` entered with `target_ey=0.82 m`, but the final published
command was immediately replaced by a wall admission hold.

## Earliest violated invariant

The direct controller clears `current_prediction` after claiming normal execution authority. The
same-cycle final wall admission requires that prediction to prove the executed path. The execution
owner therefore destroys its own mandatory certificate input.

## Propagation

```text
direct activation
  -> MPCC solve bypassed
  -> prediction cleared
  -> Dynamic Escape lease still marked fresh
  -> final monitor assigns prediction ownership to Dynamic Escape
  -> prediction-unavailable
  -> retained solution unavailable
  -> wall hold and non-positive acceleration
  -> next-cycle Cruise/direct ownership churn
```

## Evidence

- Source activation: `mpc_controller_cpp.cpp`, stopped local-path direct-entry block.
- Source destruction: `low_speed_shift_control()` clears both prediction vectors.
- Consumer: `evaluate_predicted_path_wall_metrics()` rejects an empty prediction.
- Runtime: replay lines 532-538, decision 1944.
- History: direct latch introduced in `a3929d7` (`make gate2対策`).

## Existing patch relationship

Wall admission is behaving correctly: it refuses to certify an unavailable trajectory. The direct
entry feasibility guard added later only decides when the obsolete controller may enter; it cannot
repair missing execution provenance. Loosening the wall classifier or adding another grace/lease
would mask the producer defect.

## Root cause

Stopped-vehicle planning and stopped-vehicle command ownership were coupled in a compatibility
bypass. After Dynamic Escape MPCC became the intended execution owner, the old direct owner was not
retired, leaving two incompatible execution/certificate contracts in one control cycle.
