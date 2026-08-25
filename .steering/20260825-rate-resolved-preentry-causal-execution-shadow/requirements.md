# Requirements

## Objective

Prove a causal asynchronous six-state Overtake execution producer after the
left/right tactical worker has selected a homotopy. The tactical worker may
provide side and Mission geometry, but its old trajectory may not be retained
as an executable artifact.

## Root cause entering this Slice

- Tactical left/right evaluation completes 0.20--0.47 s after its snapshot.
- Track/Follow continues publishing while it runs, so the old trajectory no
  longer shares the committed steering/velocity predecessor.
- Rebuilding the selected side from the current state can certify it, but a
  synchronous full tactical rebuild costs 88--115 ms and violates 40 Hz.

## Constraints

- Shadow only; no Mission mutation, normal command, or production store.
- No parameter, threshold, timeout, lease, flag, or fallback change.
- Reuse the tactical result only as a selected-side homotopy hint.
- Deep-copy the current isolated snapshot in the callback, then rebuild the
  prospective problem outside the callback without running the tactical
  candidate search or five-state solver again.
- Bind the six-state request to the steering command committed in the current
  cycle, then solve outside the callback with latest-only semantics.
- Seal target, side, generation, stage geometry and physical world provenance.
- Keep Emergency/Recovery and ROS interfaces unchanged.
- Do not stage `aichallenge/result-summary.json`.

## Acceptance

- Current-state prospective-problem build time and async solver time are
  measured independently.
- Submitted identity records the current decision and committed predecessor.
- Results with a conflicting side, target or Mission generation fail closed.
  A temporarily unavailable current tactical selection may run physical
  current-world proof in shadow, but is never counted as current tactical
  authority.
- Source contract proves the shadow result cannot reach Mission admission,
  production plan store or command publication.
- Build and package tests pass.
- Bounded `make dev2` establishes callback p95/max, solve completion rate,
  result age and current-world join outcome.
