# Requirements: Slice 6 reachable preview horizon (rejected experiment)

## Purpose

Remove the structural normal-authority hole observed in
`output/20260827-194608/d1/autoware.log` without adding another fallback or
relaxing the physical wall proof.

## Observed failure

- A certified Cruise/Follow plan drove normally with a four-stage horizon of
  about 0.35 s while the vehicle was still accelerating at about 3.6 m/s.
- The vehicle was already near `e_y=1.6 m`, but the narrowing wall envelope a
  few metres ahead was outside that shortened horizon.
- When the narrowing reached stage zero, the fresh semantic adapter rejected
  the measured `e_y=1.55852 m` against `[-3.82518, 0.856278] m`.
- The retained plan simultaneously failed exact continuation with
  `invalid-lateral-bounds`, leaving no canonical normal authority.
- Emergency braking preserved safety authority, but the vehicle was already at
  the wall and AWSIM recovery reported mixed wall contacts.

## Tested hypothesis

The spatial reference schedule might have been discarded merely because its
`dt` was derived from a target velocity that the vehicle had not reached yet.
The experiment therefore minimally dilated stage durations to a schedule
reachable under the same acceleration and lag-state bounds used by the
canonical MPCC.  Dynamic evidence rejected this as a production repair.

## Constraints

- Keep the canonical seven-state rate-resolved MPCC as the only normal owner.
- Keep Emergency and stuck Recovery external to normal authority.
- Do not change wall margin, OSQP tolerances, acceleration limits, or safety
  proof acceptance.
- Do not restore a multi-second sparse horizon at standstill.  If the next
  spatial stage requires more than the formulation maximum stage duration, the
  horizon must still truncate.
- The resolved stage schedule must be used by dynamics, the semantic request,
  exact physical reconstruction, and the immutable execution artifact.

## Experiment acceptance criteria

- Unit tests cover unchanged race-speed schedules, stopped truncation, and an
  accelerating case whose wall preview is preserved by time dilation.
- Package build and all package tests pass.
- In dev2 evidence, Cruise/Follow at intermediate speed retains materially more
  than the previous four-stage / 0.35 s preview when the spatial horizon is
  physically schedulable.
- No regression in physical proof rejection typing, stale authority rejection,
  or normal/Emergency ownership.

## Outcome

Rejected.  The longer schedule increased the intermediate-speed preview, but
the same change caused fresh Cruise wall-refined QPs to reach maximum
iterations in the single-car Gate.  It therefore did not preserve the clean
Track/Cruise invariant and was reverted before commit.
