# Follow retained current-world design

## Observed cause chain

```text
Follow fresh QP is occasionally unavailable
-> Follow has no retained canonical candidate
-> canonical selector has no normal same-formulation source
-> a future Follow production promotion would choose Emergency Stop
```

Track/Cruise already closes the equivalent gap with current-world retained revalidation. That proof
is intentionally empty-world-only:

```text
active_vehicle_count != 0 -> DynamicObstaclePresent
```

Reusing it for Follow would either reject every useful retained Follow plan or require weakening the
obstacle proof. Neither is acceptable.

## Alternatives

### A. Tune OSQP until fresh coverage reaches 100%

Rejected. Earlier numerical experiments changed symptoms, did not prove physical correctness, and
some worsened solve behavior. Solver availability and executable authority are separate contracts.

### B. Continue the old command for a short age lease

Rejected. Age does not prove that the target, wall, cursor, or physical gap remains valid. This is
the exact hidden fallback pattern prohibited by Phase 0.

### C. Reuse empty-world retained proof and ignore the Follow target

Rejected. It would certify only walls while the plan's defining dynamic constraint is unproved.

### D. Extend current-world proof with a typed Follow target tube

Selected. It preserves the same immutable plan/cursor/proof/selector architecture while making the
dynamic object an explicit current input.

## Pure contract

Add a `FollowDynamicObstacleObservation` containing:

- target identity and observation generation/time;
- current/fresh semantic state;
- hard gap;
- current-relative target elapsed times and relative progress;
- a deterministic tube fingerprint.

The fingerprint is derived from every field that changes the obstacle proof. The generic retained
proof continues to carry the fingerprint and current target generation, so later candidate
construction cannot substitute a different observation.

For each retained sample at relative time `t`:

```text
target_absolute_progress(t)
  = lifted_current_ego_origin + interpolate(target_relative_progress, t)

retained_ego_physical_progress(t)
  = retained_state.progress + retained_state.lag

gap(t)
  = target_absolute_progress(t) - retained_ego_physical_progress(t)
```

The stage is obstacle-clear only when `gap(t) >= hard_gap`. The same current target progress is used
for the delay prefix and control-to-retained connector at `t=0`; they cannot bypass a current hard
gap violation.

Wall proof remains the existing swept-footprint proof over the measured prefix, connector, and
remaining retained horizon. No geometric margin is relaxed.

## Runtime lifecycle

1. Complete the existing fresh Follow solve/certificate/canonical command chain.
2. Only after that chain succeeds, atomically replace a dedicated Follow plan store.
3. On a later eligible cycle with no fresh canonical command, snapshot the stored plan.
4. Resolve its exact cursor and current course-frame window.
5. Build the target tube from the current accepted Follow longitudinal contract.
6. Build current wall + target proof.
7. Build retained candidate, run the canonical selector, extract exact actuation and prediction.
8. Record `selected=0`; this Slice remains shadow-only.

Failed fresh post-extraction checks never poison the retained store. A retained plan from another
intent/target/generation cannot cross the proof boundary.

## Logging

Follow shadow telemetry gains typed retained fields:

- attempted/world-certified/candidate/authority/actuation;
- stored/retained plan IDs;
- target tube generation and fingerprint;
- current-world reason and retained proof/candidate/authority reasons;
- minimum retained gap and rejected stage;
- concise retained detail.

These distinguish solver unavailability from target change, gap closure, wall rejection, cursor
expiry, and selector rejection without adding decision branches.

## Deletion boundary

This Slice adds no production fallback and therefore deletes no production owner. Follow scalar
ownership may be deleted only in the authority-promotion Slice after fresh+retained dynamic evidence
and explicit authority approval.

