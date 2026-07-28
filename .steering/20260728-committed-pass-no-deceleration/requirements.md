# Requirements

## Purpose

Prevent a target-related speed drop after a physically committed Pass. On this narrow race
course, a transient deceleration lets the passed vehicle close the opening and block the ego
again.

## Evidence

In `output/20260728-094334/d1/autoware.log`, the successful P1 Pass showed:

- `ShiftOut -> Pass` and front-cap release at `target_s=3.76 m`;
- current lateral separation remained `1.87 m` (`lat_clear=1`, `body_clear=1`);
- a transient line-target error made `lateral_complete=0`;
- the front cap was reapplied for about `0.95 s`, setting `v_ref=3.70 m/s`;
- ego speed dropped from about `5.58 m/s` to `4.05 m/s`;
- the cap then released again and the maneuver completed with `Pass -> Return`.

The deceleration was not caused by a body-overlap risk. It was caused by requiring the current
line goal to remain complete after Pass had already latched physical separation.

## Required behavior

- Do not change pre-Pass `ShiftOut` speed policy.
- Initial Pass speed release still requires the full inflated `1.50 m` lateral clearance and the
  existing line-completion conditions.
- Once Pass speed release is active, keep it active without requiring `lateral_complete` every
  cycle when:
  - phase remains `Pass`;
  - full-clearance exclusion has latched;
  - locked target is still valid;
  - current lateral separation remains at least the physical combined body width (`1.45 m`);
  - the execution path remains physically feasible;
  - the actual footprint is not in wall contact.
- Do not let the locked target's speed cap reduce the Pass reference in the `1.45-1.50 m` band.
- Existing curve/domain/global speed limits, another vehicle, body-overlap risk, wall contact,
  physical path infeasibility, and emergency protection retain authority.

## Acceptance criteria

- An already released Pass at body-clear separation remains released when
  `lateral_complete=false`.
- The same state does not remain released below the body-clear threshold.
- An unlatched Pass or ShiftOut cannot acquire release through this hold policy.
- Unit tests and `make autoware-build` pass.
