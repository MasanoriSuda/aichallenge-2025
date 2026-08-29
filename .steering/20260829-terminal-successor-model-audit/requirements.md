# Requirements

## Objective

Explain why a current-world-admitted ShiftOut artifact loses authority at
`terminal-contingency-unavailable` shortly after publication, and restore one
coherent contract between the certified terminal successor and the Stop
command that production actually publishes.

## Frozen failure

Run `output/20260829-164506`, D1 episode 2, sequence 1074:

- retained proof is initially Accepted;
- decision 1710 rejects with `terminal-contingency-unavailable`;
- the terminal Stop wall proof reports `collision` after 196 checked poses;
- the current normal continuation remains clear for its current-stage prefix;
- external Stop then publishes max braking with `lateral=track-reference-path`;
- later steering/progress rejections occur only after Stop owns the wire.

## Constraints

- Do not relax wall clearance, lateral acceleration, continuity or solver
  tolerances.
- Do not add a lease, grace period, timeout, resume rule or fallback.
- Do not allow a partial normal prefix without a physically certified terminal
  successor.
- The certified successor and the serialized Stop controller must have one
  explicit lateral-control contract.
- Preserve external Stop authority and current interface contracts.

## Acceptance

- The exact lateral policy assumed by terminal contingency proof is logged and
  structurally tied to the policy production will publish.
- A certified partial prefix cannot be followed by a different unproved Stop
  steering policy.
- Genuine wall-infeasible normal-plus-Stop combinations remain rejected.
- Build, focused tests, complete package tests and dynamic `make dev2` pass.
