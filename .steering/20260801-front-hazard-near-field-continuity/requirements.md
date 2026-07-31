# Requirements

## Purpose

Prevent a stopped or slow V2X target from slipping through the front-hazard protection when its
classification briefly changes from front to side/rear near a hairpin.

## Evidence

- In `output/20260801-082537`, the target was first detected as a stopped/slow front hazard and
  SafetyBrake was entered.
- At close range the target temporarily moved out of the front classification, the existing
  0.25 s hold released, and the target reappeared almost at zero longitudinal distance.
- The episode was followed by repeated solver failures and a stuck-recovery cycle of roughly
  36 seconds.

## Constraints

- Preserve the existing short 0.25 s hold setting; do not introduce a fixed long braking tail.
- Refresh the hold only for the same target while it is physically near and its inflated lateral
  footprint still conflicts with the ego vehicle.
- A target that is truly rear-clear or laterally clear must not block racing or overtaking.
- Do not change ROS topics, messages, launch contracts, `result-summary.json`, or the user's
  `wp_id_offset` setting.

## Definition of Done

- Rear-clear classification is course-aware when course projection is available.
- A physically close, laterally conflicting held target cannot be released only because the
  local tangent reports it behind the ego vehicle.
- Unit tests cover the hairpin seam, true rear-clear, lateral-clear, and invalid-observation cases.
- The package's existing unit test suite passes.
