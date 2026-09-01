# Requirements

## Objective

Promote only the offline-admitted 3.0 m-gated safe-speed formulation to an
explicit four-vehicle runtime A/B mode.

## Constraints

- Keep the production `fixed_lidar_brake` default unchanged during the A/B.
- Keep the base and spatial steering artifacts byte-identical.
- Do not add a creep state, timer, lease or retained escape command.
- Preserve the existing 3.0 m slow and 1.5 m hard-stop exposure boundaries.
- Never assume more deceleration than the configured published brake command.
- Missing or stale wheel speed may not produce positive acceleration inside
  the slow zone.

## Definition of Done

- Pure safety tests cover clear, hard-stop, bounded restart and stale speed.
- Core and launch/runtime provenance expose an explicit control mode.
- The package builds and all relevant tests pass.
- A four-vehicle A/B is classified against the frozen packaged baseline.
