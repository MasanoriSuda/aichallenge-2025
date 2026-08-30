# Requirements

## Baseline

- Source HEAD: `f018e653 audit(mpcc): prove seven-state terminal stop feasibility`
- Frozen failure: decision 4017 in `output/20260830-200852`.

## Proven defect

Every fixed-lateral maximum-braking Stop fails the unchanged physical proof,
while a causal seven-state Stop succeeds.  Production has not changed.

## Objective

Determine whether the already-certified normal trajectory supplies a
sufficient low-cost varying lateral candidate for the same Stop controller.
The candidate maps local course progress to lateral reference and preserves
the existing maximum-braking, steering-rate, wall, peer and publisher
contracts.

## Constraints

- Observation only until frozen proof succeeds.
- No additional solver, worker, Store, fallback or authority.
- No parameter, clearance, tolerance, timeout or Mission change.
- Exact nonlinear wall/dynamic proof remains the acceptance owner.
- If promoted later, replace the fixed target; do not retain both candidate
  families as permanent fallbacks.

