# Audit: rejected reachable-preview experiment

## Result

Rejected and reverted.  No production or test code from this experiment is
present in the worktree.

## Evidence

- `output/20260827-200729` (dev2) showed that bounded time dilation changed an
  intermediate-speed Follow horizon from roughly 3--5 stages to 20 stages.
- The same run still lost fresh/retained authority.  A representative Follow
  solve failed at the wall-refined QP and the retained join later rejected the
  current state.
- `output/20260827-201723` (single-car dev) removed V2X and tactical planning
  from the experiment.  Fresh Cruise wall-refined QPs still reached maximum
  iterations near 7.8 m/s and normal authority was lost.

The second observation is the decisive falsification: preserving the spatial
preview did not preserve the already-clean Track/Cruise Gate.

## Conclusion

The short executable horizon can amplify a later authority loss, but it is not
the sole root producer.  The next investigation must start from the unmodified
baseline `output/20260827-194608` and identify why a currently published
execution intent is no longer replenished before its certified cursor expires.
Changing horizon length, wall margin, solver tolerance, or fallback behavior is
outside this rejected Slice.
