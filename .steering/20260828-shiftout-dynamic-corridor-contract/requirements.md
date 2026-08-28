# Requirements

## Objective

Determine why a certified seven-state ShiftOut which still had wall and body
clearance was invalidated by `live overtake corridor unavailable`, then repair
the responsible producer/consumer contract without weakening physical proof.

## Frozen evidence

`output/20260828-230302/d1/autoware.log` entered ShiftOut for target `d2`,
published certified and executed-retained seven-state ShiftOut artifacts, and
later reported:

```text
current_ey=0.43, goal=1.04
robust_footprint_clear=1
physical_footprint_clear=1
corridor_min=6.24 m
live execution corridor hold released: corridor=blocked
ShiftOut -> FollowPrepare
reason=dynamic Mission wait: live overtake corridor unavailable
```

At the same boundary, the V2X gap diagnostic reported both pass-side gaps as
unavailable while the canonical dynamic-obstacle QP was still producing
stay-behind rows and no pass-side rows.

## Constraints

- Do not change wall/vehicle clearance, timeout, lease, grace, solver
  tolerance, cadence or speed parameter.
- Do not make the live gap planner a second physical certificate.
- Do not let an old Mission override a current hard wall/contact fault.
- Distinguish physical infeasibility from a topology/admission/lifecycle
  signal before changing production behavior.
- Preserve seven-state single normal authority and exact published artifact
  identity.

## Exit criteria

- The producer of `overtake_execution_corridor_blocked` is traced to its exact
  geometry, time and target inputs.
- Its result is compared with the certified artifact, current target tube and
  runtime physical wall/body proof at the same decision.
- A root-cause statement identifies whether the failure is Mission lifecycle,
  candidate generation, single-SQP, model/certificate mismatch or physical
  infeasibility.
- Any code change has one-to-one tests and preserves all hard faults.
- Build, full package tests and a bounded dynamic acceptance pass.
