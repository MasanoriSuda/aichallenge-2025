# Design

## Observed causal chain

```text
LowSpeedAvoidance local path is feasible
  -> low_speed_shift_control_active_ = true
  -> get_control() returns before every MPCC solve
  -> LowSpeedDirect computes one steering/speed pair
  -> current_prediction is cleared
  -> final Dynamic Escape wall admission requires that prediction
  -> prediction-unavailable
  -> wall hold/deceleration
```

The wall guard is the downstream detector, not the producer of the defect. Relaxing the wall guard
or synthesizing a prediction for the hand-written controller would preserve two normal authorities
and hide the architecture mismatch.

## Historical origin

`LowSpeedDirect` was introduced by `a3929d7` as a Gate2 stopped-vehicle bypass before the current
five-state MPCC, connected Dynamic Escape profile and physical wall certificate existed. Its latch
was deliberately designed to own the complete pass and hand control back only after a probe solve.
That compatibility ownership is now obsolete.

## Target flow

```text
stopped vehicle / low-speed local path
  -> gap planner chooses corridor and target
  -> MpcProblem carries bounds/reference and Dynamic Escape intent
  -> progress MPCC solves lateral + longitudinal command together
  -> solved prediction is physically certified
  -> final wall admission consumes the same prediction
```

## Implementation boundary

1. Remove the local-path block which activates direct normal control.
2. Remove the early `LowSpeedDirect` return from `get_control()`.
3. Remove the solver-failure branch which reactivates direct normal control during handoff.
4. Preserve planning inputs and the existing MPCC/fail-closed paths.
5. Leave dead compatibility state/functions for the final Slice 6 deletion only if removing them
   would mix broad cleanup with this authority correction. They must be statically unreachable.

## Why this is not a parameter change

No feasibility threshold changes. The fix prevents an old controller from discarding the exact
evidence required by a newer safety contract. Any remaining MPCC infeasibility will therefore be
visible as an MPCC problem/solver/certificate failure instead of being masked by direct control.

## Remaining migration boundary

After this Slice, stopped-front handling uses the live progress-MPCC execution path, but that path
still contains five-state conversion and three-state/legacy cycle fallback. Slice 5 must replace
that fallback chain with fresh certified canonical solution, bounded same-formulation retained
solution, then Emergency Stop.
