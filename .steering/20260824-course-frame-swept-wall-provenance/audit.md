# Audit

## Initial evidence

- The rejected exact solution's discrete stage footprints were all clear;
  otherwise validation would have returned `hard-wall-contact` before the
  swept check.
- The failure was `swept-path-violation` for the segment ending at stage 15.
- The QP constrains lateral state at stages and with progress-aligned wall rows,
  but has no explicit swept-footprint constraint between stages.
- `evaluate_clear_footprint_path` linearly interpolates world x/y/yaw.
- The failure diagnostic currently replaces the actual interpolated collision
  pose with the safe segment endpoint, masking the distinction.

## Current classification

- Root cause: not yet accepted.
- High-confidence detection defect: rejected substep pose/fraction is lost.
- Competing structural causes: inappropriate world chord versus missing
  continuous wall feasibility in the QP.

## Dynamic evidence

Run `output/20260824-065336` eventually reached Overtake entry near waypoint
171. The entry-side tactical artifact was certified with 20 exact five-state
stages and promoted `Idle -> ShiftOut`. On the immediately following control
cycle, however, the canonical Overtake worker had only a pending job:

- entry: `certificate=1`, `exact_stages=20`;
- canonical worker: `submitted=1`, `pending=1`, `completed=0`;
- live proof: `exact five-state trajectory contract incomplete`;
- action: `entry-rollback`, with no command from an exact Overtake solution.

Consequently the new `course_sweep` comparison was not exercised. The run is
not evidence for either competing continuous-wall hypothesis.

## Accepted conclusion for this Slice

Only the provenance defect is accepted here. The generic swept-footprint
validator now retains the actual rejected interpolated pose and the segment
substep/fraction. When a later exact five-state world-chord rejection occurs,
the controller also evaluates an observation-only course/Frenet-resampled
trajectory under the same footprint and grid. Neither comparison changes the
admission result or published command.

The earliest observed break is now upstream: tactical entry authority is
promoted before an exact, current-world canonical Overtake execution artifact
is available. The structural repair belongs in a separate Slice and must make
promotion atomic with canonical readiness; permitting a one-cycle legacy hold,
age lease, retry, or timeout would only hide the authority gap.
