# Validation

## Static candidate validation

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40 test targets,
  1805 tests, zero errors, failures, or skipped tests.
- Candidate unit tests proved that an immutable plan with an unreachable
  steering sequence was rejected.

Static success was not treated as authority for production acceptance.

## Dynamic A/B

Candidate artifact: `output/20260824-232452`.

Both domains remained at `wp_id=29`:

- Domain 1: 2 Track and 15 Cruise Emergency publications, predominantly
  canonical QP solve failure with no retained plan;
- Domain 2: 2 Track and 15 Cruise Emergency publications, with
  `canonical-plan-reject/unpublishable-steering-transition` and no retained
  plan.

This is a hard regression relative to `output/20260824-230215`, where both
vehicles ran and Domain 2 had no Follow Emergency publications. The run was
stopped immediately with `make down`.

## Conclusion

The logs confirm a real solver/publisher time-base mismatch, but restricting
every coarse stage endpoint to one publish-period change is not the repair.
The coarse stage describes evolution over its own duration and cannot also be
the instantaneous 40 Hz command sequence. Canonical execution needs an
explicit rate-resolved actuation representation before the downstream live
continuity check can be eliminated as a recurring authority gap.

All candidate source and test changes were removed. Only this rejected-
hypothesis audit remains. `aichallenge/result-summary.json` is intentionally
excluded.
