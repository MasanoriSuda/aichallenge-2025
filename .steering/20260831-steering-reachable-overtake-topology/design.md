# Design: steering-reachable Overtake topology

## Root cause

The production population represented immediate, fixed-midpoint and late
disjunction timing. It did not represent the first temporal homotopy that the
measured steering actuator can physically establish. A previous frozen world
needed midpoint stage 9, but decision 1833 needs stage 6; replacing one fixed
fraction with another would repeat the same defect.

For the frozen decision-1833 source:

```text
current steering       = -0.172680 rad
left steering limit    = +0.366519 rad
steering rate limit    =  0.731707 rad/s
yaw response delay     =  0.130000 s
reachability duration  =  0.867 s
cumulative stage time  =  stage 6
```

The independently accepted physical-diagonal F arm reaches full-side at the
same stage. This establishes a physical producer for the missing topology.

## Change

Add one `SteeringReachablePhysicalDiagonal` member. Its full-side stage is the
first state whose cumulative immutable stage durations cover:

```text
abs(side_steering_limit - measured_current_steering) / steering_rate_limit
+ yaw_response_time_constant
```

The stage is also required to satisfy the existing diagonal minimum length and
planning horizon. If it duplicates the midpoint member it is not added twice.

The population order is:

1. direct side;
2. steering-reachable physical diagonal;
3. midpoint physical diagonal when distinct;
4. finite-boundary or late-exact topology.

This is a bounded temporal-homotopy population, not a clearance parameter or
a special decision-1833 branch. Evaluation still stops at the first fully
certified candidate.

## Rejected alternatives

- Change stage 9 to stage 6: snapshot-specific threshold patch.
- Relax exact wall proof: invalid geometries would gain authority.
- Add SQP iterations: C/F already solve with the unchanged single SQP.
- Remove midpoint or late-exact: both have separate frozen and dynamic
  acceptance evidence.
- Enumerate the full audit lattice live: violates the bounded async compute
  contract.
