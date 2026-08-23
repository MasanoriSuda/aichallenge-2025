# Design

## Observed mismatch

The exact stage states are reconstructed on the solved course frame and checked
individually. The subsequent swept proof passes only their world poses to
`evaluate_clear_footprint_path`, which interpolates x/y/yaw linearly. On a
hairpin, that chord can cut inside the reference curve. The diagnostic then
reports `swept_path[rejected_path_index]`, which is the segment endpoint rather
than the interpolated pose that touched the wall.

## Hypotheses

1. World-chord interpolation creates a false collision inside a curved segment.
2. The solver truly creates a wall-crossing Frenet transition even when both
   endpoints are clear.
3. Both interpolations fail, meaning the QP needs a continuous wall constraint.

## Investigation sequence

1. Make generic swept-path diagnostics preserve the actual rejected pose and
   substep fraction.
2. Build a pure helper that densifies five-state segments by interpolating
   solved Frenet state/progress and sampling the course frame.
3. Add a curved-course test where endpoint chord and course-following sweep
   deliberately differ.
4. Run both proofs in observation mode against the same exact artifact.
5. Promote course-frame resampling only if the observed rejection is chord-only;
   otherwise retain the current guard and move continuous constraints upstream.

## Non-goals

- This Slice does not change tactical side choice, Mission state, Recovery or
  controller authority.
- It does not make a rejected trajectory executable merely because its stage
  endpoints are clear.
