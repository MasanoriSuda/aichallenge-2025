# Design

## Root cause

The current Follow path resolves one speed limit from the current front
distance and copies it to every MPC stage. The solver therefore cannot identify
which future target observation and gap produced each longitudinal constraint,
and the configured following distance is not represented as a predicted state
contract.

The first dynamic run exposed an earlier boundary defect: the orchestrator can
retain the `Follow` action while no vehicle is physically in front. In that
state `target_vehicle_id` may name a side vehicle, while `front_distance` and
`front_speed` are infinite. Treating the action label alone as a Follow target
made shadow construction report a misleading configuration failure. Shadow
eligibility now requires one coherent front observation from the same behavior
cycle. The retained action remains production-owned and is not reclassified in
this Slice.

## Contract

`FollowLongitudinalContractRequest` contains the current intent, immutable
target identity/generation, observation age/maximum age, current course-relative
target progress, target speed, desired and hard gaps, the existing speed-margin
policy, base horizon progress/velocity references and stage durations.

For state stage `k` at elapsed time `t[k]`:

```text
target_progress[k] = current_target_progress + target_speed * t[k]
desired_ego_progress[k] = target_progress[k] - desired_gap
hard_ego_progress_upper[k] = target_progress[k] - hard_gap
```

The desired progress is intersected with the ordinary racing reference. The
hard upper bound is intersected with the course horizon upper bound. Future
progress lower bounds belong to the Follow formulation and permit holding at
zero relative progress; monotonicity remains enforced by non-negative virtual
progress speed.

The velocity reference uses the existing signed distance-gain policy evaluated
on the predicted base-reference gap. It is a stage reference/bound input to the
same MPCC, not a second command owner.

## Identity

Follow problem fingerprints must include the current target ID, target
observation generation and a Follow-specific bounds/cost schema. A stale or
missing target produces no contract.

An invalid target distance/speed produces `invalid-target-kinematics`, while
invalid gap/speed-policy values produce `invalid-configuration`. These reasons
must not be merged because the former is an observation/provenance boundary and
the latter is a controller setup defect.

## Promotion boundary

This Slice is shadow-only. The legacy scalar Follow cap remains production
authority until a later Slice proves dynamic coverage and explicitly deletes
that owner in the same promotion change.
