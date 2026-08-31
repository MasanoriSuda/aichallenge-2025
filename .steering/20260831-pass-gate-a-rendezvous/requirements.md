# Requirements: Pass Gate-A rendezvous

## Objective

Repair the asynchronous ShiftOut-to-Pass handoff observed in
`output/20260831-100351/d2` episode 2 without weakening physical or
current-world proof.

Frozen evidence:

- `ShiftOut` starts at `1788138273.993299038`;
- certified current-world Pass proposals exist at tactical decisions 1540,
  1564 and 1808;
- ShiftOut completion is observed around waypoint 72, but the proposal in that
  callback is empty;
- the phase remains ShiftOut for about 8.38 seconds and finally enters Recovery
  because the locked target is stale/lost;
- the late decision-1808 Pass proposal is solver-solved, physically accepted,
  dynamically clear and authority-ready.

## Root-cause gate

ShiftOut completion and an asynchronous Pass proposal are independently valid
events. Requiring both to occur in one control callback is not a valid atomic
handoff contract. The completion boundary must be monotonic for the exact
encounter identity while every Pass trajectory and certificate remains freshly
rebuilt from the current world.

## Constraints

- do not retain path samples, corridor geometry or a Pass certificate;
- do not add a Mission resume rule, lease, grace period, timeout or fallback;
- do not change solver settings, wall margin, clearance or velocity policy;
- reset the boundary fact on target/generation/side replacement and phase exit;
- production authority still requires a complete, current-world certified
  Pass proposal in the consuming callback.

## Definition of done

- a focused test proves completion and proposal may arrive in different
  callbacks for the same encounter;
- wrong identity and phase reset cannot reuse the boundary fact;
- all package tests pass;
- a bounded dynamic run no longer loses an otherwise certified Pass solely due
  to the one-cycle rendezvous;
- remaining failures are classified independently.
