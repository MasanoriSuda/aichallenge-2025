# Requirements

## Problem

In `make dev3`, P2 detects the staggered grid vehicle as a close stopped front target and enters
`SafetyBrake -> Follow`. The visually open side corridor is not used until the normal 5 m overtake
entry distance is recovered.

## Requirements

- During the configured start-grid grace only, a latched stationary front target may trigger an
  immediate side breakout when the existing all-vehicle inflated gap planner finds a valid
  corridor. A delayed side classification must not prevent the first evaluation.
- Expiring the entry grace must not cancel a same-target ShiftOut/Pass already in progress.
- Before side lock, both collision-inflated corridors must be evaluated and the wider feasible
  corridor must take priority over the initial stagger direction.
- A validated breakout line must not be cancelled solely by the close-front risk metric that the
  breakout arbitration intentionally bypassed.
- A validated breakout must not receive the generic locked-target front-speed cap a second time
  from OvertakeLine after the dedicated breakout speed policy has selected its reference.
- The normal race-wide 5 m overtake entry guard must remain unchanged.
- A missing or invalid side corridor must retain the existing emergency brake behavior.
- A new breakout must not apply when the feature is disabled, outside the grace period, or to a
  non-latched target. A breakout continued after grace must remain the same active line target.
- The transition must be visible in the V2X debug log.

## Definition of Done

- Unit tests cover breakout eligibility and fail-closed cases.
- `multi_purpose_mpc_ros` builds and its tests pass.
- A subsequent `make dev3` shows P2 entering `Overtake` with reason `start-grid breakout` instead
  of first entering `SafetyBrake` or `Follow`, provided a side corridor is feasible. Its
  OvertakeLine debug must also show the breakout cap released instead of remaining at front speed
  plus 0.5 m/s.
