# Design

## Root cause

`FollowLongitudinalContract` produces a current target prediction at the
current five-state problem's stage timestamps. Current-world retained proof
then samples that vector at timestamps owned by a different, previously solved
canonical plan. Adaptive/relinearized stage durations make those time domains
different. When the retained terminal time is slightly longer, the sampler
returns `TargetHorizonUnavailable` even though current target kinematics are
available and the physical gap is safe.

The visible Emergency is downstream and correct under the incomplete proof.
The upstream defect is failure to build current obstacle evidence for the
artifact being certified.

## Repair

1. Preserve target speed explicitly in the Follow longitudinal contract.
2. Add a pure current-observation coverage function. Given the required
   retained terminal time, it:
   - validates the current observation and target speed;
   - returns the original samples when already covered;
   - otherwise appends one terminal sample using the same constant-velocity
     model as the current Follow contract;
   - recomputes the immutable tube fingerprint.
3. In retained Follow proof preparation, determine the exact terminal time
   from the already validated retained execution window before sealing the
   obstacle tube.
4. Continue using the existing proof builder and hard-gap certificate without
   relaxation.

## Rejected alternatives

- Increase horizon length/configuration: does not establish equal time domains
  and is parameter tuning.
- Clamp a query to the last target sample: silently changes a moving target to
  stationary without recording the prediction model.
- Ignore the terminal stage: weakens full-horizon certification.
- Accept the retained plan on target age alone: bypasses current-world proof.

## Non-scope

- Actual `stage-gap-violation` handling.
- Async plan activation/steering continuity.
- Overtake target corridors.
- Parameter tuning.
