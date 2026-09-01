# Design

## Authority boundary

The existing frozen production spatial command remains the base command.  In
the B experiment only, the verified peer-512 recurrent correction is applied
on the same current sample:

```text
published steering = clip(
  spatial production steering + clip(recurrent correction, +/-0.24 rad),
  physical steering limits)
```

Failure to produce a current finite correction preserves the already-valid
spatial production command.  No recurrent state or correction is retained as
command authority across a failed admission.

## A/B order

1. Reuse the immutable authority-disabled single and peer runs as A.
2. Run one single-vehicle B Gate with explicit authority.
3. Only after every single Gate passes, run the deterministic three-vehicle B
   Gate with domains 1 and 2 unchanged MPC peers and domain 3 as the student.
4. Compare domain 3 with the authority-disabled peer baseline.

The authority-disabled async executor is not used in B.  It is an observation
isolation mechanism and cannot provide a fresh current-sample control result.
Therefore B must independently pass the runtime Gate under peer CPU load.

## Decision boundary

- Any Finish, penalty, stall, stale/error/reset or minimum scan-rate Gate
  failure rejects bounded peer512 authority.
- A run that passes safety but materially regresses lap time provides no
  promotion evidence.
- One passing peer run can admit only a repeated authority A/B; it cannot
  change packaged defaults by itself.
- No parameter or model retraining is part of this Slice.
