# Design

## Scope

This is a local admission-order fix.  It does not alter tactical side
selection, target prediction, MPCC weights, wall margins, or Recovery.

## Pass-entry contract

At the ShiftOut completion boundary, construct the lateral profile that would
own execution:

1. a currently authoritative DP prefix;
2. otherwise a connected solved-MPCC bridge trajectory;
3. otherwise the committed same-side Pass goal.

Evaluate that profile from the measured `e_y`, heading error, and speed with
the existing static-footprint and lateral-acceleration horizon evaluator.
Admission requires:

- the existing fresh target prediction checks;
- no physical/static-wall horizon failure;
- no wall-margin degradation or wall clamp;
- no lateral-acceleration projection.

If the execution preflight fails, feed it into the existing Pass-entry
physical gate.  That gate retains ShiftOut and builds a bounded current-side
replan prefix.  Its existing time/distance expiry still selects a new Mission
or Recovery instead of waiting forever.

## Why this is local

The previous flow admitted Pass before the physical execution horizon was
evaluated later in the same callback.  Moving the full optimizer/state machine
would be high risk.  Reusing the current evaluator at the admission boundary
removes the contradiction without changing authority ownership elsewhere.

## Diagnostics

Gate logs distinguish `wall_warning` from `execution_preflight` and include
the preflight reason and maximum required lateral acceleration.
