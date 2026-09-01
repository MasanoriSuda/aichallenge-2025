# E2E DAgger v3 NPC-authority requirements

## Objective

Test whether v3 fixes the dynamic-obstacle direction-transition failure that
ended the v2 NPC authority run at a wall.

## Frozen comparison

- scenario: `e2e-npc-single`, one learned ego plus two runtime NPCs;
- candidate3 remains the embedded base;
- v3 is the only spatial candidate;
- authority uses the unchanged `0.12 rad` runtime bound;
- no parameter, safety-layer, launch-default or checkpoint change;
- v2 failure reference: `output/20260901-180313`.

## Acceptance

- ego finishes 3/3 laps;
- ego penalty and stall counts are zero;
- runtime inference and provenance gates pass;
- authority remains bounded and fresh;
- no wall lock corresponding to the v2 direction-transition failure.

Failure freezes the run for causal extraction.  It does not authorize bound
tuning or a hand-authored avoidance branch.
