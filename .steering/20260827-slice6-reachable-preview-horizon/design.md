# Design: physically scheduled spatial preview (rejected)

## Causal chain

The legacy trajectory is spatially sampled, while its stage `dt` is computed
from the desired speed.  During acceleration, desired speed can be much higher
than measured speed.  The previous resolver treated those optimistic times as
fixed and truncated at the first spatial reference beyond

```text
v0 * t + 0.5 * a_max * t^2 + lag_bound
```

This made the wall horizon shortest exactly while the vehicle needed advance
notice of a narrowing corridor.

## Resolution

For each spatial stage:

1. accumulate the reference distance;
2. compute the minimum cumulative time needed to reach
   `max(0, reference_distance - lag_bound)` under `v0` and `a_max`;
3. set the stage duration to the greater of the nominal duration and the
   additional time needed to meet that cumulative time;
4. accept it only when it is no greater than the canonical maximum stage `dt`;
5. truncate at the first stage which cannot be represented within that bound.

The resulting durations replace the optimistic legacy durations throughout the
extended MPCC construction.  No state box, wall bound, solver tolerance, or
physical proof is relaxed.

## Why this is not parameter tuning

The change repairs a unit/semantic mismatch between a spatial horizon and a
temporal dynamics model.  It does not alter competition aggressiveness.  A
schedule already reachable at its nominal `dt` is bit-for-bit unchanged.

## Alternatives rejected

- Increasing `N` or wall lookahead alone: the resolver would still discard it.
- Expanding the stage-zero wall bound: hides a physically outside state.
- Retaining the old plan longer: exact continuation correctly rejected it.
- Adding a new fallback after Emergency: treats the downstream symptom and
  leaves the short-preview wall collision intact.
- Forcing all stages to maximum `dt`: restores the sparse multi-second stopped
  horizon which the original reachability gate was intended to prevent.

## Dynamic obstacle note

The current target time tube is produced before the extended schedule is
resolved.  Time dilation therefore makes the target longitudinal samples
conservative: the target is represented no farther ahead than before while the
ego progress dynamics use the longer physical time.  This Slice does not claim
complete uncertainty-tube support; that remains a later quality item.

## Falsification

The experiment did preserve more stages at intermediate speed, but it did not
repair the authority invariant.  In `output/20260827-201723`, with no V2X
target present, fresh Cruise problems at about 7.8 m/s reached the OSQP
iteration limit inside the wall-refined problem and normal authority was lost.
The change therefore traded a short executable prefix for a larger, less
reliably solvable QP.  It was reverted in full; none of this design owns
production behavior.
