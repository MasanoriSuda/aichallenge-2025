# Design

## 1. Publish after shadow evaluation

`assess_side(..., true)` creates `mpcc_receding_mission`; ordinary side
assessment does not.  Publish the locked-side shadow assessment's feasible DP
candidate after both shadow branches have been assessed.  This output remains
DP-reference-only and is not an atomic Mission replacement request.

## 2. Refresh source fallback

Evaluate rolling refresh admission in this order:

1. current-state receding prefix;
2. complete same-side Mission.

Only a source that passes the existing refresh resolver is committed.  Thus a
stale, malformed or interval-ineligible prefix cannot suppress a valid complete
Mission path.  Rejected sources leave the currently executing path unchanged.

## 3. Runtime wall contraction rebase

The wall contraction preflight already validates a transition from current
`e_y` to the contracted goal over the selected shift distance.  Convert that
same distance-domain transition into a DP execution path and store it on the
replacement candidate.  The path holds the contracted goal after the shift;
Return remains owned by the existing Return phase.

The builder is kept in `v2x_overtake_core` so numeric validation and transition
shape can be unit-tested without ROS.

## Expected dynamic evidence

- At least one `DP execution rolling refresh ... source=receding_prefix` line.
- DP path age resets while the shadow log reports a feasible locked-side
  prefix.
- A wall-center contraction does not leave DP execution without a path aimed
  at the contracted goal.
- No increase in actual-wall hard failures.
