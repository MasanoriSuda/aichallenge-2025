# Track/Cruise retained revalidation design

## Root cause

The existing retained candidate contract can be satisfied by a plan ID, a stage-index window and
three physical-certificate booleans.  Those values do not identify the current ego observation,
geometry, obstacle observation, control pose, or the progress/time samples that were checked.
Consequently an old stage `k` can be accidentally paired with current stage `k`, even though the
two refer to different physical points and prediction times.

## Selected correction

Keep fresh certification unchanged and introduce a distinct retained-proof boundary:

1. Derive an immutable remaining execution window from the plan and exact cursor.
2. Lift current measured progress onto that window's unwrapped branch.
3. Require one current safety evaluation for every exact retained endpoint.
4. Require separate delay-prefix and first-connector certificates.
5. Seal all current provenance, cursor values, samples and safety results into a fingerprint.
6. Let a retained candidate builder independently recompute and validate that fingerprint against
   the current provenance.

The pure layer does not decide how ROS/world data are sampled.  It defines the identities and axes
which a runtime adapter must use, preventing same-index substitution.  Runtime wall and obstacle
adapters will populate typed evaluations from current providers before any shadow candidate exists.

## Runtime adapter boundary

The runtime adapter reconstructs the retained state in the current course frame and evaluates
three distinct physical paths against the current static-wall occupancy grid and current vehicle
footprint:

1. measured ego pose to the exact predicted control pose;
2. predicted control pose to the interpolated retained-current pose;
3. every remaining retained stage segment.

The control-pose path, course-frame knots and obstacle observation receive independent
fingerprints.  Gate B is intentionally limited to Track/Cruise with a fresh, explicitly observed
empty V2X world.  `NoData`, a stale observation, any active peer or any target fails closed.  The
raster wall checker proves non-intersection only, so its certified metric-clearance lower bound is
recorded as zero and is not reused as a tuning margin.

The controller snapshots the previous canonical plan before attempting the fresh solve.  It calls
the retained adapter only when fresh canonical authority is unavailable.  A successfully extracted
retained actuation is counted in shadow telemetry only; it is never copied into the pending
publisher actuation.

## Why this is not another fallback

The retained plan is neither legacy MPC nor an unverified last command.  It is the remaining
portion of the same five-state canonical formulation, and it is unavailable unless it receives a
new current-observation physical proof.  No old certificate is reused and no final command source
is added in this Slice.

## Failure-first matrix

| Case | Expected result |
|---|---|
| old stage/current stage share index but not progress | reject sample identity |
| cursor 60 ms into a 100 ms stage | first duration 40 ms, endpoint `k+1` |
| current ego generation differs | reject current provenance |
| target ID same but obstacle generation differs | reject current provenance |
| stage geometry changes without new proof | reject current provenance |
| retained endpoints clear but first connector blocked | reject physical proof |
| delay prefix blocked | reject physical proof |
| obstacle crosses at an intermediate current-relative time | reject stage evaluation |
| retained progress crosses lap seam | accept one explicit lift only |
| missing control pose/course frame/tube/path length | reject missing provenance/input |

## Promotion boundary

Passing these tests and runtime shadow observation permits only a Gate B report.  Connecting a
retained candidate to the final normal publisher remains an authority promotion and requires an
explicit user approval in a later Slice.

The first single-car runtime run exercised the retained call site during transient fresh rejects,
but the environment provided V2X `NoData`, not an explicit empty observation.  The retained proof
therefore rejected safely.  This is useful fail-closed evidence, not evidence sufficient for
publisher promotion.
