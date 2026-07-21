# Requirements

## Problem

In `make dev3`, P2 detects the staggered grid vehicle as a close stopped front target and enters
`SafetyBrake -> Follow`. The visually open side corridor is not used until the normal 5 m overtake
entry distance is recovered.

## Requirements

- During the configured start-grid grace only, a latched stationary grid target may trigger an
  immediate side breakout when the existing inflated-vehicle gap planner finds a valid corridor.
- The normal race-wide 5 m overtake entry guard must remain unchanged.
- A missing or invalid side corridor must retain the existing emergency brake behavior.
- The breakout must not apply when the feature is disabled, outside the grace period, or to a
  non-latched target.
- The transition must be visible in the V2X debug log.

## Definition of Done

- Unit tests cover breakout eligibility and fail-closed cases.
- `multi_purpose_mpc_ros` builds and its tests pass.
- A subsequent `make dev3` shows P2 entering `Overtake` with reason `start-grid breakout` instead
  of first entering `SafetyBrake` or `Follow`, provided a side corridor is feasible.
