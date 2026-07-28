# Design

## Committed Pass speed hold

Extend `OvertakeFrontCapReleaseRequest` with an explicit
`committed_pass_speed_hold_allowed` input. The controller sets it only when the current state has
already passed all of these gates:

- `Pass` phase;
- front-overlap exclusion latched;
- front-cap release already active;
- locked target currently body-clear;
- no target position jump;
- physically feasible execution horizon;
- no actual footprint wall contact.

The pure release policy may then treat the existing release as laterally complete even if the
directional line-goal test temporarily becomes false.

This is a hold-only path. It cannot perform the initial release because both the controller guard
and pure policy require the previous release state.

## Clearance thresholds

- Initial/full-speed acquisition: `1.50 m`
  (`vehicle_radius 1.45 + prediction_margin 0.05`).
- Committed speed-release hold: `1.45 m` physical combined kart width.
- Below `1.45 m`: no committed hold; generic front-risk protection may apply.

Change `v2x_overtake_pass_front_cap_reapply_lateral_clearance` from `1.50 m` to `1.45 m` so the
release latch matches the physical body-clear boundary.

## Observability

Add `speed_hold` to the existing low-rate `OvertakeLine debug` record. It is true only when the
new committed hold is actively bridging a false `lateral_complete` result.
