# Requirements

## Objective

Prevent an admitted MPC/MPCC overtake path from retaining control after its optimizer
source is stale, its closed-loop tracking has diverged, or the extended solver has
fallen back repeatedly.

## Scope

- `multi_purpose_mpc_ros` overtake DP execution authority
- MPC/MPCC execution-reference trust bounds
- extended-progress fallback handoff
- focused pure-core and controller tests
- `docs/spec/mpc-integration.md` current implementation contract

## Constraints

- Keep ROS topic, service, message and launch contracts unchanged.
- Do not change `aichallenge_system/` or generated `output/` artifacts.
- Preserve the user's existing `aichallenge/result-summary.json` change.
- Treat this as a 2025 AWSIM-derived simulator control policy, not a confirmed 2026
  competition specification.

## Acceptance criteria

1. Runtime validation cannot extend a DP path beyond its configured absolute age.
2. A DP execution reference outside the current Mission/closed-loop trust envelope is
   not sent to MPC/MPCC.
3. Extended solver degradation cannot silently continue the same aggressive reference
   without a validated execution fallback.
4. Unit tests cover stale-path rejection and execution-reference trust contraction.
5. The package builds and focused tests pass.
