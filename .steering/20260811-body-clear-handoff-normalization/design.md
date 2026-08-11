# Design

## 1. Normal ownership transfer

Extend the pure handoff resolver with an `ordinary_pass_ownership_latched`
input and a typed release reason. A normal lateral/front-cap latch terminates
the handoff immediately; the normal Pass policy then owns Behavior and speed.

## 2. Live deadline contraction

At runtime, estimate hard-gap TTC from the current locked-target longitudinal
distance and closing speed. The effective deadline is:

```text
min(frozen_absolute_deadline,
    now + max(0, live_hard_gap_ttc - configured_deadline_margin))
```

An unavailable or non-closing live estimate leaves the frozen deadline
unchanged. The live estimate never extends it.

## 3. Mission/speed separation

The handoff may continue to preserve the admitted Mission and suppress a
prediction-only SafetyBrake. When prediction is invalid or predicts a body
sweep overlap, a pure speed policy limits the OvertakeLine velocity reference
to the lower of current speed and target speed plus the existing unlatched Pass
closing allowance. It does not introduce a new hard MPC state constraint.

## 4. Diagnostics

Record the effective remaining time, whether the deadline was live-TTC
contracted, whether the speed hold is active, and the typed release reason.

## Impact

Only `multi_purpose_mpc_ros` overtake core/controller and its unit tests change.
No interface or parameter-file change is required.
