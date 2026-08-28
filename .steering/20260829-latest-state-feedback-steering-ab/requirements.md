# Requirements

## Objective

Determine whether an AS-RTI-style bounded feedback phase can reconnect a
prepared seven-state MPCC candidate whose desired steering is no longer
reachable from the last actually published command.

## Frozen evidence

- `output/20260829-025035` reported 26 d1 and 2 d2 asynchronous candidate
  rejections with `steering-unreachable`.
- The on-trajectory comparison produced no exact parent/candidate state join.
- The existing continuation model can replay an immutable MPCC suffix from a
  fresh physical state, but production returns before that proof when the
  candidate command is outside the publication-to-publication slew envelope.

## Invariants

- This Slice is observation-only; it cannot construct or publish authority.
- The feedback steering is the exact minimizer of distance to the prepared
  candidate steering under the existing actuator/rate envelope.
- No tolerance, clearance, rate, timeout, lease, fallback or Mission rule is
  changed.
- The same immutable candidate suffix and nonlinear model are used.
- Production continues to reject the original unreachable candidate.

## Exit criteria

- Every steering-unreachable candidate is classified as feedback-projectable
  or feedback-invalid.
- A projectable result is replayed through the existing nonlinear continuation
  proof and classified without affecting authority.
- Dynamic evidence establishes whether the actuator/model connector closes
  before implementing full wall and dynamic-obstacle certification.
