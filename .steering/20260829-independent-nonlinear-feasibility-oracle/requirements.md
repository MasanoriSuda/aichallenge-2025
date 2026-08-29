# Requirements: independent nonlinear feasibility oracle

## Objective

Classify the frozen ShiftOut and Cruise failures after the dense OSQP owner
A/B proved that local solver conditioning is not causal.

## Comparison contract

- Reuse the immutable production snapshot, input bounds and initial state.
- Integrate the seven-state nonlinear dynamics at the physical 10 ms proof
  cadence.
- Retain physical state, wall, steering-prefix, progress-wall, swept-wall and
  dynamic-obstacle constraints.
- Exclude only the post-hoc lag and heading affine buckets already identified
  as a model trust-region device rather than a physical certificate.
- Verify every feasible external primal with the unchanged C++ exact wall,
  dynamic-obstacle, terminal-successor and Stop-suffix proof chain.

## Constraints

- Observation-only; no Store, mailbox, publisher or production authority.
- No solver tolerance, iteration, clearance, Mission or configuration change.
- Use deterministic multi-starts and report the owning violated constraint
  when no feasible point is found.
- A nonlinear optimizer success flag is not certification; only the unchanged
  C++ proof chain may accept a bundle.

## Definition of done

- A constrained nonlinear oracle independently evaluates ShiftOut and Cruise.
- The output names the worst physical constraint and maximum required slack.
- Feasible primals are passed through the existing external-primal proof CLI.
- Each failure is classified as single-SQP/model mismatch, candidate defect,
  or still-unresolved physical infeasibility.
- Production source and parameters remain unchanged.
