# Follow retained current-world requirements

## Purpose

Close the remaining same-formulation availability gap before Follow can be considered for production
authority. A fresh five-state Follow solution is not available on every eligible control cycle. The
normal authority contract must therefore be:

```text
fresh certified Follow canonical plan
-> retained Follow canonical plan re-certified against the current target and wall
-> Emergency Stop
```

This Slice is shadow-only. It must not connect Follow to the final publisher, change driving
parameters, relax a physical constraint, or reintroduce legacy MPC as a fallback.

## Root-cause boundary

The fresh positive gate demonstrated a complete canonical Follow chain on 90.18% of valid live
attempts. A fresh-only promotion would turn the remaining unavailable cycles into Emergency Stop.
The existing retained current-world implementation cannot close this gap because it deliberately
accepts only an explicitly observed empty dynamic world and rejects any active target.

The missing capability is not another solver policy. It is a current-world proof for the already
solved Follow plan using the current coherent observation of the same target.

## Required invariants

- Retained execution uses the immutable canonical plan and exact non-clamping cursor.
- Current intent and intent generation match the stored plan.
- Current target ID matches the stored problem target ID.
- Current target observation is coherent, fresh, and has a nonzero generation.
- A typed target-tube fingerprint seals target ID, observation generation/time, hard gap, and the
  current stage-wise target progress forecast.
- The retained ego trajectory is evaluated at current-relative stage times against that target tube.
- Physical ego progress is `theta + e_lag`; the hard gap is checked using that value.
- The measured-to-control prefix, connector, and complete remaining retained horizon are checked
  against the current wall.
- A target change, stale/malformed tube, insufficient current gap, stage gap violation, wall block,
  progress discontinuity, cursor expiry, or identity mismatch fails closed.
- No old certificate, age-only lease, warm-start vector, or retained label is executable authority.

## Non-goals

- Follow production promotion or scalar-owner deletion.
- Hold/Stop integration.
- Overtake behavior changes.
- OSQP setting, wall margin, gap, velocity, acceleration, or cost tuning.
- A new fallback/timeout/feature flag.

## Exit gate

- Pure tests cover acceptance and fail-closed target/gap/wall/provenance cases.
- Follow fresh plans are stored only after the complete fresh canonical command chain succeeds.
- When fresh Follow is unavailable, retained shadow evaluation runs through the same canonical
  selector and exact actuation extraction.
- Package tests and `make autoware-build` pass.
- Dynamic evidence shows a retained Follow selection after a typed fresh miss, or explicitly records
  that the run did not contain the required event. Production authority remains unchanged.

