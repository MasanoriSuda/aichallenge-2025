# Audit record

## Baseline

- branch: `develop_july`
- commit: `46c853b`
- code state: Overtake fresh and retained canonical selections are shadow-only
- production state: legacy Overtake conversion/three-state fallback remains
- user work preserved: `aichallenge/result-summary.json`

## Evidence boundary

This Slice begins with no source/config modification. The first AWSIM closed-loop
run after `46c853b` is the sole causal timeline for the live gate.

## Closed-loop result

- Run: `output/20260824-005436`
- Scenario: `make dev2`
- Outcome: **invalid Overtake gate**. Neither vehicle produced a usable
  closed-loop Overtake interval before an upstream normal-authority failure.

Domain 2 exposed the failure most clearly:

- Track/Cruise canonical solves reported `solved` on 261 sampled outcome
  transitions;
- 153 of those were rejected by the execution-primal boundary;
- rejected fields were acceleration (133), predicted velocity (16), and
  virtual-progress speed (4);
- final output therefore entered `canonical-normal-emergency-stop` 184 times.

The earliest failure occurred before the race session was active at decision
550. OSQP returned `solved`, but virtual-progress speed was `-0.00155846 m/s`
against the exact non-negative input row. The row-local violation was
`0.00155846`, above its `0.00100156` tolerance. A later common failure was
acceleration `1.37329 m/s2` against the `1.37 m/s2` upper bound, with
`0.00329009` violation above `0.00237329` row tolerance.

Domain 1 also became unusable, but through two already typed upstream gaps:
Follow asynchronous production remained pending during early curve-gated
Follow, and later Track/Cruise produced three virtual-progress execution-primal
rejections. These are not evidence about Overtake retained continuity.

## Root-cause separation

The visible stopped vehicle and later Frenet displacement are downstream
effects. The earliest causal break is the Track/Cruise canonical solver-context
contract:

```text
Track/Cruise five-state problem contains mixed physical units
-> its dedicated solver context uses the default global-scale residual policy
-> OSQP result is admitted as solved using the largest QP row scale
-> a small-unit actuator/input identity row exceeds its own tolerance
-> strict execution-primal boundary correctly rejects the result
-> no fresh canonical plan exists
-> retained proof cannot bridge the start/current-world discontinuity
-> explicit emergency authority publishes zero speed / braking
```

The repository already has a formulation-scoped
`RowToleranceNormalized` policy. Follow canonical and the live extended
Overtake solver use it, while the separately constructed Track/Cruise solver
context does not. This is an initialization/contract omission, not a reason to
relax OSQP tolerances, execution bounds, wall margin, or certificate age.

The earlier rejected experiment in
`.steering/20260822-osqp-rowwise-residual-admission` changed the shared solver
boundary and broke the legacy three-state curvature-rate row. That
counterexample does not apply to assigning the already accepted row-normalized
policy only to the dedicated five-state Track/Cruise canonical context.

## Gate decision

- Overtake production promotion remains blocked.
- The retained Overtake implementation is neither accepted nor rejected by
  this live run; the requested evidence was never reached.
- Select a new bounded Slice that restores the Track/Cruise five-state solver
  context to the canonical row-tolerance contract and proves it dynamically.
- Do not add fallback, timeout, lease, flag, or parameter tuning.
