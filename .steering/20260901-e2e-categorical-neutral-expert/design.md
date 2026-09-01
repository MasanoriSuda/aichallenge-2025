# Design

The existing signed-mixture head already exposes three direction logits and two
side magnitudes.  Production v11 composes a continuous residual from softmax
probabilities, so small left/right probability mass can steer on normal track
and the same probability also attenuates a required large correction.

This Slice uses the existing `categorical_expert` training objective and exact
`winner_take_all` evaluator contract:

- neutral argmax emits exactly zero;
- left argmax emits the learned negative magnitude;
- right argmax emits the learned positive magnitude.

This is a learned mode selection, not a hand-written runtime trigger.  Training
uses the current-distribution normal corpus and sample-balanced contract from
v12; no weighting sweep is allowed.  The candidate remains offline until both
normal behavior and the immutable failure suffix pass.

If it passes, a later integration step must add the decode contract to the
checkpoint/runtime identity and test shadow mode before authority.  It must not
silently reinterpret the current v11 artifact with winner-take-all decoding.
