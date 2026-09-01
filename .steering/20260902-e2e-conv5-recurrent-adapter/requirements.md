# Requirements

## Objective

Build an offline-only temporal steering candidate from the admitted
speed-committed teacher evidence without changing production authority.

## Evidence-bound requirements

- Preserve the packaged production controller and its ROS publisher.
- Keep both the embedded TinyLidarNet and the packaged v11 spatial authority
  immutable, and verify both exact tensor identities during evaluation.
- Replace the lossy 10-dimensional `fc3` recurrent input with the projected
  `conv5` representation selected by the held-out seed-2033 probe.
- Preserve causal run boundaries and causal wheel-speed synchronization.
- Train with independent, certified production-normal sequences whose target
  correction is exactly zero.
- Select checkpoints using both teacher validation and independent-normal
  validation; a teacher-only improvement is insufficient.
- Do not connect the candidate to runtime or shadow authority in this slice.

## Acceptance

- Existing compact recurrent checkpoints remain loadable.
- The untrained adapter is bit-exact with the complete packaged production
  output, not merely its raw TinyLidarNet submodel.
- Train and validation sequence identities are disjoint.
- The candidate improves held-out material steering over the frozen base.
- Independent-normal correction leakage is bounded.
- All outputs are finite and bounded.
