# Requirements

## Objective

Compare the frozen distance-only LiDAR safety authority with continuous
speed-aware alternatives on the exact failed and clean four-vehicle bags
before changing production runtime.

## Constraints

- Keep lateral ML authority and all production defaults frozen.
- Reconstruct the qualified `0.8 m/s2`, `4.6 m/s` pace request.
- Use the same percentile frontal-clearance contract as runtime.
- Report false/intervention burden on clean d3/d4 as well as lead time on
  failed d1/d2.
- Treat every replay result as counterfactual, not closed-loop admission.

## Definition of Done

- Current fixed policy reproduces recorded acceleration closely enough to make
  the replay meaningful.
- Effective-deceleration assumptions 1, 2 and 3 m/s2 are compared without
  silently changing commanded braking.
- A runtime candidate is selected only if it intervenes before the frozen
  failure and does not globally dominate clean driving.
