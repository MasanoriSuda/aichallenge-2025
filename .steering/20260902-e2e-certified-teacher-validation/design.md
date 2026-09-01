# Design

Replay the unchanged `precontact_teacher` in `e2e-npc-single` with seed 2032.
This is a policy-level validation rather than a frame split: no sample from the
same physical rollout may cross from train into validation.

If the run passes, use the strict relabel command with its own competition
analysis and `--split val`.  The new outcome certificate and source run identity
must differ from seed 2031 while the teacher mode and base checkpoint SHA remain
identical.

No model is trained in this steering until the second source is admitted and
its correction distribution is inspected.
