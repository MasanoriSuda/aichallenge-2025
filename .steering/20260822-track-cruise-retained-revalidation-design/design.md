# Retained revalidation design

## Source audit

### What is already correct

- `CanonicalExecutionPlan` stores complete predicted states, absolute progress, acceleration,
  curvature, virtual progress speed and exact stage duration
  (`canonical_execution_plan.hpp:18-43`).
- `resolve_execution_cursor()` consumes elapsed stage durations and returns the exact first
  remaining stage plus partial-stage elapsed time; it expires instead of repeating the last
  command (`canonical_execution_plan.cpp:142-180`).
- Fresh Track/Cruise certification reconstructs solved world poses at solved absolute progress,
  checks the oriented footprint at every state and checks the swept path from a current pose
  (`mpc_controller_cpp.cpp:20265-20595`).
- The course-frame sampler requires finite strictly increasing knots and fails closed outside
  its provenance window (`mpc_stage_geometry.cpp:73-145`).
- The sealed solver problem already distinguishes ego observation, stage geometry and target
  observation generations (`mpcc_execution_contract.hpp:173-189`,
  `mpc_controller_cpp.cpp:23322-23365`).

### Where the proof is lost

`CanonicalExecutionRevalidation` carries a decision ID, plan ID, stage-index window and a
`PhysicalCertificate` only (`canonical_execution_plan.hpp:156-163`).  Candidate construction
checks those numbers and three booleans, then copies them into the candidate
(`canonical_execution_plan.cpp:326-376`).

It does not carry or validate:

- current ego observation generation;
- current target-obstacle generation;
- current stage/course-frame geometry identity;
- measured or predicted control pose identity;
- delay-prefix proof;
- the absolute-progress/time sample sequence;
- a fingerprint of the inputs which produced the physical certificate.

The current shadow caller is safe only because it certifies a fresh solution and immediately
uses the fresh cursor in the same callback (`mpc_controller_cpp.cpp:21286-21472`).  That is not
a retained revalidation implementation.

## Root cause and causal chain

```text
retained plan advances by elapsed time
-> current wall/obstacle arrays are rebuilt for a new stage geometry and observation
-> revalidation contract identifies only the old cursor's stage number
-> an implementation can accidentally pair old stage k with current stage k
-> static geometry is sampled at the wrong progress and obstacle occupancy at the wrong time
-> checked/wall_clear/obstacles_clear can describe a different physical horizon
-> selector may accept a stale retained command as current normal authority
```

The root cause is not a conservative parameter.  It is the absence of a typed, sealed
current-observation proof whose temporal and spatial axes match the retained command window.

## Hypotheses and falsifiers

| Hypothesis | Supporting evidence | Falsifier | Confidence |
|---|---|---|---|
| Reusing the old physical certificate is unsafe | The old certificate is bound to the old problem fingerprint and observations | Existing revalidation recomputes world footprint and obstacle occupancy from current observations | High |
| Reusing current stage `k` bounds for retained stage `k` is unsafe | Current stage zero moves with tracking waypoint; retained states store absolute progress | A typed alignment proves equal absolute progress and relative time for every pair | High |
| Static wall and dynamic obstacle checks need different axes | Wall map/course frame are spatial; obstacle prediction is generated with per-stage horizon time (`mpc_controller_cpp.cpp:4268-4355`) | Obstacles are proven static for the full retained horizon | High |
| The first sweep may start at old predicted state `k` | Cursor records partial elapsed but revalidation has no current pose | Existing proof starts at the delay-compensated current control pose | High |
| Lap seam can alias progress | MPCC progress is locally unwrapped while measured course progress can reset; warm start treats discontinuity specially | Current progress is globally monotonic across laps | Medium-high |

## Rejected approaches

### A. Reuse the original solve certificate

Rejected.  It proves only the old observation, bounds and pose.  A certificate's remaining
validity time is not proof that the world stayed unchanged.

### B. Slice current `lb/ub` arrays at the retained cursor index

Rejected.  It conflates three identities: old command time, current horizon stage and absolute
course progress.  It also evaluates a moving obstacle at the wrong relative time.

### C. Always discard retained plans and require a fresh solve

This is safe but removes the bounded last-certified solution required by the Slice 3 fallback
contract.  It turns every transient solver miss into Emergency Stop and does not meet the intended
single-formulation continuity.

### D. Typed dual-axis current revalidation

Selected.  Build a pure revalidation request from the current observation, exact retained
cursor, current course-frame/wall provider and current dynamic-obstacle tube.  Produce one sealed
proof or one explicit reject reason.  Connect it only after fresh Gate A evidence.

## Proposed pure contracts

Names are illustrative; implementation may refine names without weakening the identities.

```cpp
struct RetainedStageSample
{
  std::size_t control_stage_index;
  double relative_time_sec;       // from current observation
  double segment_duration_sec;    // partial first stage, then full stages
  double absolute_progress_m;     // retained solved progress, unwrapped
  CanonicalPredictedState state;  // endpoint state k + 1
};

struct CurrentExecutionProvenance
{
  std::uint64_t decision_id;
  ControlIntent intent;
  std::uint64_t observation_generation;
  std::uint64_t stage_geometry_id;
  std::uint64_t target_obstacle_generation;
  std::uint64_t control_pose_id;
  double observation_sec;
  double path_length_m;
};

struct RetainedExecutionProof
{
  CurrentExecutionProvenance current;
  std::uint64_t plan_id;
  CanonicalExecutionCursor cursor;
  std::uint64_t proof_fingerprint;
  PhysicalCertificate physical;
};
```

The fingerprint must cover the current provenance, exact cursor including
`stage_elapsed_sec`, stage samples, control-pose identity, course-frame/window identity and
obstacle-tube identity.  `PhysicalCertificate` remains a summary, not the proof identity.

## Revalidation data flow

```text
current decision + current intent
current ego observation + measured pose
current delay-compensated command pose
current target/obstacle observations
retained immutable plan + exact time cursor
        |
        v
build remaining stage samples
  first endpoint = predicted state k+1
  first dt       = duration[k] - elapsed_in_stage
  later dt       = original stage duration
        |
        +--> lift current circular progress onto retained unwrapped branch
        |    build current course-frame coverage for every retained progress
        |    sample current wall/corridor geometry by progress
        |
        +--> predict current obstacle tube by cumulative relative time
             compare occupancy at the corresponding progress/lateral state
        |
        v
prove measured -> predicted-control delay prefix
prove predicted-control -> every retained endpoint swept path
        |
        v
seal RetainedExecutionProof
        |
        v
candidate builder verifies exact plan/cursor/current-decision/proof fingerprint
```

### Two-prefix rule

The command will act from a delay-compensated predicted pose, while the vehicle is observed at a
measured pose.  Neither may be silently substituted for the other.

1. Check the measured-to-predicted delay prefix using the current measured motion estimate.
2. Start the retained horizon sweep at the predicted command-application pose.

If either proof is unavailable or blocked, reject retained authority.  This avoids both a gap in
physical coverage and double-counting the already elapsed portion of stage `k`.

### Circular progress rule

For a circular path, choose the unique integer path-length offset which places current progress
on the retained plan's continuous branch and within an explicit continuity tolerance.  Record the
offset in the proof.  Ambiguous/missing path length, more than one admissible branch or an uncovered
query rejects.  Do not use modulo inside the sampler and do not extrapolate course-frame knots.

### Dynamic obstacle rule

Current gap-planner bounds are produced using a per-stage prediction time and current V2X age
(`mpc_controller_cpp.cpp:4288-4355`) before being intersected into current lateral bounds
(`mpc_controller_cpp.cpp:17673-17713`).  A retained proof must therefore rebuild or sample the
current obstacle tube at each retained sample's cumulative relative time.  It may not reuse the
original plan's `obstacles_clear` flag or the current horizon's same-numbered stage.

## Failure-first tests for the implementation slice

1. **Stage-index alias:** old stage 1 is progress 101 m, current stage 1 is 109 m.  Both index
   bounds look safe; progress-aligned sampling must reject or use the 101 m bound.
2. **Partial first stage:** cursor is 60 ms into a 100 ms stage.  First sample duration must be
   40 ms and endpoint must be state `k+1`; state `k` may not be emitted as current.
3. **Ego generation mismatch:** a proof produced for decision/observation `n` is presented under
   `n+1`; candidate construction rejects it.
4. **Obstacle generation mismatch:** target ID is unchanged but V2X generation advances;
   retained proof is rejected until recomputed.
5. **Geometry mismatch without alignment:** current geometry fingerprint changes while retained
   progress remains covered; only a newly sealed progress-aligned proof can pass.
6. **Current connector collision:** all retained discrete endpoints are wall-clear but the
   predicted-control-pose to first endpoint sweep crosses a wall; reject.
7. **Delay-prefix collision:** predicted control pose is clear but measured-to-predicted prefix
   crosses/starts in a wall; reject.
8. **Moving obstacle crossing:** wall path is clear, but the current obstacle tube intersects at
   `t=0.24 s`; reject even if current stage index bounds are open.
9. **Circular seam:** retained progress crosses one lap boundary.  The explicit lap lift succeeds
   once; ambiguous or uncovered lift rejects.
10. **Missing input:** absent control pose, course-frame coverage, current tube or path length
    rejects and selector resolves Emergency Stop, never legacy MPC.

## Planned implementation order after Gate A

1. Add pure stage-sample and provenance/fingerprint types.
2. Add the failing deterministic tests above.
3. Implement exact partial-stage sampling and circular progress lift.
4. Implement pure current wall/obstacle proof adapters without runtime authority.
5. Extend candidate construction to require the sealed proof identity.
6. Run focused/full tests and build.
7. Connect retained **shadow** candidate production and collect telemetry.
8. Request explicit approval before final publisher promotion.

## Complexity and deletion audit

- New normal authorities: zero.
- New runtime flags: zero.
- New grace/retry/clamp branches: zero.
- Existing old-certificate reuse: not currently connected; it must remain prohibited.
- Existing current-stage-index shortcut: not currently implemented; this design prevents its
  introduction.
- Final Track/Cruise legacy branch deletion remains Gate C work, after A/B evidence.
