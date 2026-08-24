# Design

## Time model

For one control callback:

- `observation_sec`: callback/observation time used for V2X freshness.
- `control_origin_sec`: time represented by the latency-predicted MPC state.
- `prediction_delay_sec = control_origin_sec - observation_sec`.
- retained stage times start at `control_origin_sec`.

The asynchronous solve may complete before `control_origin_sec`; completion
wall time therefore must be compared with capture time, not prediction origin.

## Ego prefix

The current latency predictor already emits a sequence from raw odometry pose
to the execution pose.  Replace the anonymous pose vector inside the controller
with a typed prefix carrying a monotonically nondecreasing elapsed time for each
pose.  Existing five-state proofs consume only its pose projection.  The
six-state retained proof consumes poses and elapsed times together.

## Dynamic timeline

Dynamic-obstacle positions are normalized to `observation_sec`.  Revalidation
checks:

1. measured pose to control pose over `0 .. prediction_delay_sec`,
2. control pose to expected retained pose at `prediction_delay_sec`,
3. retained suffix from `prediction_delay_sec` onward.

This prevents a moving peer from being frozen while the ego advances through
latency compensation.

## Fail-closed invariants

- `control_origin_sec >= observation_sec`.
- artifact `prediction_origin_sec >= identity.snapshot_sec`.
- completion time is not required to be later than a future control origin,
  but remains later than capture time.
- prefix pose/time vector sizes match, start at zero, are monotonic, and end at
  exactly `control_origin_sec - observation_sec`.
- the prefix endpoint equals `control_pose`.

## Rejected alternatives

- Keep the old timestamps because the fixed delay cancels: this leaves dynamic
  proof on the wrong time axis and breaks if delay policy changes.
- Inflate obstacle radius by speed times delay: hides time semantics in a
  margin and double-counts or undercounts depending on direction.
- Shift only the cursor by delay: ego suffix becomes correct, but the delay
  prefix still freezes moving obstacles.

## Authority

This Slice remains `authority=shadow, selected=0`.  Its output is evidence for
the later atomic six-state promotion/legacy-owner deletion Slice.
