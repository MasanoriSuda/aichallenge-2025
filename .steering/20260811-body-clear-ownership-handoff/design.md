# Design

## 1. Runtime body-clear handoff

Add a pure resolution function that converts the frozen candidate timing into
a runtime state:

- `active`: admitted Mission, finite deadline, before expiry, no confirmed
  current overlap.
- `satisfied`: current body footprints are already separated (diagnostic; it
  does not extend the handoff).
- `expired`: execution passed the predicted hard-distance time without a valid
  handoff.
- `remaining_sec`: diagnostic time to expiry.

The handoff always expires at the hard-distance time, even if separation was
observed; by then the ordinary Pass latch/release must own execution. The
controller stores an absolute expiry at Mission freeze time using the
candidate's `predicted_hard_distance_time_sec`. Mission replacement gets a new
expiry because its rollout is generated from the replacement time; the overall
Mission total-time clock remains preserved.

## 2. Longitudinal ownership across phase boundary

Extend committed Pass Behavior ownership with
`body_clear_handoff_active`. Before the ordinary Pass latch/release exists, the
validated fixed line may retain Overtake only when:

- the bounded handoff is active,
- target identity and course progress are continuous,
- current body overlap is not confirmed,
- all existing forbidden-waypoint, intrusion, emergency, and solver guards are
  clear.

ShiftOut ownership also consumes the runtime handoff state instead of the
permanently frozen feasibility boolean.

## 3. Front-danger phase handoff

Generalize the existing validated-ShiftOut exception to a validated body-clear
handoff. It may apply in ShiftOut or early Pass, but still requires current body
footprints to be separated. It only suppresses a longitudinal/prediction-only
stop; actual overlap remains fail-closed or is handled by the existing bounded
ContactContinuation policy.

## 4. SafetyBrake resume

Keep the existing direct-Pass resume geometry checks. When those checks prove
the committed side plus current and predicted lateral clearance, resume Pass
directly even if the original ShiftOut deadline has expired; that deadline's
objective is already physically satisfied. Target, corridor, wall, emergency,
intrusion, invalidation, and solver hard guards remain mandatory.

## Impact

Only `multi_purpose_mpc_ros` overtake core/controller and unit tests change.
No interface or YAML change is required.
