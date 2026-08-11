# Design

## 1. Target engagement lease

Add a target-identity lease independent of lateral Mission validity. A current
front/side target refreshes the lease. If the classifier reports no relevant
vehicle briefly, Behavior remains Follow and retains only the target ID and
entry-speed observation. The stale sample does not set `has_front_vehicle`,
does not apply a Follow cap, and cannot authorize lateral execution.

The lease is cleared immediately for an explicitly rear-side target, invalid
V2X input, or expiry.

## 2. Speed-preserving tactical revalidation

When SafeSeparation is active before rear-clear, admit a short forward lease
only while:

- the committed target is continuous;
- current body footprints are separated;
- the footprint prediction is valid;
- the current wall/corridor and solver guards are clear;
- the target remains inside a bounded local longitudinal window; and
- both lease time and traveled-distance budgets remain.

The lease reuses the existing Pass line and normal forward-escape speed
reference. It does not create a stale lateral candidate. Existing candidate
evaluation continues every control cycle, so an already preflighted alternate
can replace the Mission.

## 3. Revalidation exit

If the target is continuously clear ahead and the current/predicted footprints
plus Return corridor are clear, transition directly to Return. Do not enter
FollowPrepare. If those physical conditions are not satisfied, retain the
existing dynamic wait / FollowPrepare / Recovery behavior.

## Parameters

- `v2x_overtake_engagement_hold_sec`: 0.50 s
- `v2x_overtake_safe_separation_tactical_revalidation_enabled`: true
- `v2x_overtake_safe_separation_tactical_revalidation_max_sec`: 0.50 s
- `v2x_overtake_safe_separation_tactical_revalidation_max_distance`: 3.0 m

These bounds are deliberately short: they bridge planner/classification
chatter without turning invalid geometry into an open-ended attack.
