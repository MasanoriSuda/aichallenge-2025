# Design

## Causal chain

```text
five-state QP mixes local progress, metres, radians, m/s, m/s2 and rad/m
-> OSQP global infinity-norm termination is dominated by the largest row scale
-> OSQP reports solved while a small-unit executable box row is outside its
   own `eps_abs + eps_rel * row_scale` tolerance
-> shared wrapper keeps the raw result under its established legacy-compatible
   global contract
-> strict canonical adapter correctly rejects curvature/acceleration/velocity
-> retained proof is unavailable when obstacle observation is NoData
-> one-cycle canonical Emergency command
```

The downstream rejection is correct. Curvature stage 0 dominates, but cold
solutions also fail, so shifted warm start is an amplifier rather than the
root cause.

## Competing corrections

### A. Shared per-row rejection

Rejected dynamically in `output/20260822-234326`. It invalidated the legacy
curvature-rate row, reset the common workspace and caused a permanent cold
solve failure cascade.

### B. Scale every QP row

Rejected dynamically in `output/20260823-063519`. Although algebraically
equivalent, it destroyed ADMM convergence and moved solves to roughly 3,200
iterations / 16--22 ms while stationary.

### C. Five-state-only active-set polish

Rejected dynamically in `output/20260823-084006`. OSQP's polish phase was
enabled only for the dedicated Track/Cruise context and every result still
passed through the unchanged rowwise semantic normalizer. It completed a
46.336 s lap, but produced 27 typed execution-primal rejects: 23 curvature,
one acceleration, two predicted velocity and one virtual-progress speed.
Two failures were cold solves. This is essentially unchanged from the
26-event one-lap control and raised the observed callback maximum to
23.503 ms. The experimental source change was removed rather than retained
behind a flag.

### D. Feasible-primal restoration rollout

Next design candidate because C was dynamically falsified. Clamp only the
executable input sequence to the already-declared physical bounds, roll the
same linearized five-state dynamics from the measured initial state, and then
recheck every state/input/rate row plus the physical certificate. The restored
trajectory is a new candidate with explicit provenance, not a relabeling of
the rejected raw primal. It must fail closed if any state, rate, wall or
dynamic-obstacle contract is violated.

This changes the solver artifact more deeply and will be implemented in a
separate steering Slice with failure-first tests. It is not combined with the
rejected polish experiment.

## Failure-first experiment

Construct a two-variable mixed-unit QP:

- one small actuator variable with a tight physical box bound and an objective
  outside that bound;
- one large-scale equality row representing progress;
- default OSQP settings admit a finite `solved` result under the global norm;
- rowwise telemetry identifies the actuator row above its own tolerance.

The small QP did **not** reproduce the runtime miss: default OSQP reached a
rowwise-normalized violation of approximately `2.27e-13`. The runtime defect
therefore depends on the full horizon dynamics/conditioning and must not be
claimed from an artificial two-variable example. The test is retained only to
prove that default construction remains polish-off, dedicated construction is
polish-on, and both return a rowwise feasible result.

The failure-first integration evidence remains the 97 real
`execution-primal-reject` outcomes. Dynamic A/B is required before C can be
accepted or committed as a production correction.

## Authority and deletion audit

- No publisher or command resolver changes.
- No new authority branch.
- No legacy fallback returns.
- No YAML parameter or feature flag.
- Existing raw-primal semantic normalization remains the only execution
  boundary.

## Dynamic experiment result

Only the dedicated Track/Cruise canonical solver context was constructed with
polish enabled. Generic/legacy MPC, shared solver defaults, left/right tactical
contexts and the non-context extended solver retained default construction.
The production trace proved `polish=1` on every rejected candidate, so the
experiment was attributable and conclusively falsified. All experimental
production and test code was removed; only this evidence record remains.
