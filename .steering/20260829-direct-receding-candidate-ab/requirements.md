# Requirements

## Objective

Determine whether a freshly solved current-world seven-state MPCC candidate is
being rejected because production treats it as a successor of a persistent
executed plan, or because the candidate is already inconsistent with the
physical state when the result arrives.

## Frozen production behavior

- Do not change normal production authority.
- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not change solver tolerances, wall clearance or obstacle clearance.
- Do not promote the existing steering-projection feedback shadow.

## Comparison

For each new certified candidate observed while another plan is executing:

- A: evaluate the existing `TimeAlignedCandidate` successor contract.
- B: evaluate the same immutable candidate at cursor zero as a direct receding
  MPCC observation.
- Run the same current-state, actuator, wall, dynamic-obstacle and Follow proof
  for both A and B.
- Record result age, prediction-origin offset and the physical join residuals.

The B clock is observation-only and must be rejected by the production adapter
even if its physical proof succeeds.

## Exit classification

- A fails and B succeeds often: persistent-plan splice/lifecycle defect.
- A and B fail with a large result-age/state residual: scheduling or solve
  latency prevents direct receding adoption.
- A and B fail at the same physical proof with small age: formulation,
  candidate-generation or physical infeasibility remains upstream.
- A succeeds and B fails: the current suffix splice is necessary for that
  candidate and direct cursor-zero replacement is not yet valid.

## Deletion milestone

Delete the direct-receding observation clock, cache and telemetry after the
architecture decision is recorded. It must not become a second permanent
normal authority path.
