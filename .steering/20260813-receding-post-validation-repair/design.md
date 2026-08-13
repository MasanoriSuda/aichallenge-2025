# Design

## Observed failure

The live optimizer builds hard bounds first, but static-wall clamping and lateral-acceleration reachability are applied afterward. The adjusted path can therefore be physically usable while differing from the optimizer output, and the current implementation immediately reports `optimized horizon failed physical revalidation` or `optimized horizon escaped hard bounds`.

## Repair order

1. Revalidate the optimized lateral sequence.
2. Feed the post-validation sequence back into validation for at most three convergence passes.
3. If necessary, retry from the current planning speed downward and retain the highest feasible speed.
4. Retry with the configured hard wall clearance (`v2x_overtake_line_min_wall_clearance`) only after the robust planning clearance fails.
5. During `Pass`, release opponent separation bounds only when both current and predicted physical footprints are separated and target continuity/corridor guards remain valid.
6. If no candidate is physically feasible, retain the existing hard failure and `Recovery` transition.

The selected repair is recomputed every controller cycle. A reduced-speed repair is exposed as a temporary velocity limit; it is not latched as a new recovery mode.

## Verification

- Build the submit package.
- Run existing unit tests for `multi_purpose_mpc_ros`.
- In the next `make dev2` run, compare post-validation repair count, hard failure count, Pass completion, Recovery count, and minimum Pass speed.

