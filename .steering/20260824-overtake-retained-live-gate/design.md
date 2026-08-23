# Design

## Why a live gate is required

The saved bag replays recorded odometry from an older controller. A newly solved
canonical plan therefore cannot influence the subsequent vehicle state. This is
appropriate for deterministic contract checks but can produce progress-branch
and course-frame rejections that would not occur in closed loop.

The live gate keeps the code frozen and observes the same typed boundaries in a
closed-loop `dev2` run. It distinguishes:

- genuine fresh solver unavailability;
- genuinely expired retained certificates;
- live progress/course-frame identity defects;
- current-world corridor or wall invalidation;
- replay-only counterfactual divergence.

## Decision rule

If live uncovered cycles are dominated by cursor expiry after a long solver
gap, repair the fresh canonical producer/continuity horizon rather than extending
certificate age. If they are dominated by progress/course-frame rejection while
the vehicle tracks the plan, audit stage geometry and retained-window identity.
If coverage is complete, a separate production-promotion Slice may connect the
canonical selector and delete the legacy Overtake normal owner atomically.
