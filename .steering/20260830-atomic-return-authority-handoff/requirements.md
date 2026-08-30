# Requirements

## Frozen evidence

- Baseline: `8002feae fix(mpcc): adopt certified pre-no-return sibling`.
- Primary run: `output/20260830-120323`, Domain 1.
- At `Pass -> Return`, the geometric Return preflight is accepted and the
  tactical phase changes immediately.
- The canonical publisher then retains the previous Pass artifact because the
  proposed Return intent has no current-world authority yet.
- In another frozen episode the previous ShiftOut artifact expires before the
  asynchronous Return solve arrives, producing a one-cycle canonical Stop.
- The next Return solve is accepted and the episode completes
  `Return -> Idle`; therefore the failure is not physical infeasibility.

## Root cause

The supervisor commits `Pass -> Return` from a geometric lateral preflight,
while the canonical seven-state producer only starts solving Return after that
phase mutation.  Phase, reference ownership and executable authority therefore
cross the boundary at different cycles.

## Objective

1. Build the prospective Return problem from the current Pass world without
   mutating live tactical state.
2. Solve and physically certify Return on the existing bounded causal worker.
3. Revalidate that immutable artifact against the current world.
4. Permit `Pass -> Return` only when target, Mission generation, side, intent
   and current-world proof all match.
5. Make the same certified Return artifact available to canonical atomic
   admission in the transition cycle.

## Constraints

- Do not change wall/vehicle clearance, solver tolerance, horizon, frequency,
  weights, acceleration, braking or steering limits.
- Do not add a lease, grace period, timeout, retry, resume rule or fallback.
- Continue publishing the current certified Pass authority while a prospective
  Return artifact is unavailable.
- Do not retain a prospective Return artifact merely because the Mission
  exists; current-world revalidation and exact identity are mandatory.
- Use the existing latest-only causal worker rather than adding unbounded
  threads or a second command authority.

## Acceptance

- Unit tests reject missing, stale, wrong-target, wrong-generation, wrong-side
  and non-Return proposals.
- Unit tests admit only an exact current-world Return proposal.
- Source-contract tests prove geometric preflight alone cannot mutate phase.
- `make autoware-build` and focused/full package tests pass.
- Dynamic evidence shows `Pass -> Return` with `gate_a_joined=1` and no
  intervening canonical Stop.

