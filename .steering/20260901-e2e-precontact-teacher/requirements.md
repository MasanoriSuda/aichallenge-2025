# E2E pre-contact teacher requirements

## Objective

Replace the failed reactive side-distance teacher with a diagnostic-only teacher
that detects a physically supported side obstacle before contact and never
blends a steering command back toward that obstacle.

## Evidence boundary

In `output/20260901-090729/d4`, the student/teacher command remained directed
toward the right-side obstacle while the run slowed.  Replaying the exact scans
showed that the old 10th-percentile side aggregate stayed above its 1.8 m
trigger even when a multi-ray return was already 1.4--1.6 m away.  When it
eventually activated, linear blending reduced but did not immediately reverse
the unsafe base steering.

## Constraints

- Do not change production `fixed_lidar_brake`, checkpoint or launch defaults.
- Keep the existing `gap_teacher` behavior available for provenance.
- Expose the new policy only through an explicit diagnostic control mode.
- Reject isolated one/two-ray noise instead of changing to an unfiltered minimum.
- A supported right-side threat may not publish steering toward the right, and
  vice versa.
- Do not extract or train from the new teacher until a closed-loop run passes.

## Definition of Done

1. Unit tests prove clustered sensing, noise rejection and directional safety.
2. Offline replay activates before the recorded positive-acceleration stall.
3. An unchanged-world four-domain gate changes only d4 to the new teacher.
4. Run-level evidence decides whether the teacher can become a label source.
