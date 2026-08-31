# Requirements: Stop stationary trajectory contract

## Objective

Fix the model/certificate mismatch which rejects a physically valid maximum
braking Stop when the vehicle reaches zero speed before the current serialized
publisher interval ends.

## Frozen evidence

Run `output/20260831-184430`, decision 1116:

- normal intent was Cruise;
- the current measured-to-control path was valid and clear;
- the dynamic obstacle proof was valid and clear;
- control-origin speed was 0.055985 m/s;
- maximum braking was -3.0 m/s2;
- the 0.025 s serialized command interval therefore crossed zero speed after
  about 0.0187 s;
- the Stop successor was rejected as
  `physical-successor-rejected/exact-trajectory-rejected/invalid-path-distance`;
- normal authority was lost and the later `steering-unreachable` loop occurred
  only after Stop authority had already been selected.

## Constraints

- Do not change acceleration, wall clearance, solver tolerance, timeout,
  lease, grace or fallback policy.
- Normal Track/Cruise/Follow/Overtake trajectories must retain the strict
  forward-distance certificate.
- A repeated path distance is valid only for a Stop trajectory while adjacent
  samples are physically stationary.
- Wall and dynamic-obstacle consumers must continue to inspect the temporal
  samples; no synthetic positive distance may be invented.

## Definition of done

- A deterministic test reproduces the within-publisher zero-speed crossing.
- Stop proof accepts its stationary temporal suffix.
- The same repeated-distance sequence remains rejected for a normal or moving
  trajectory.
- Build, complete package tests and a bounded dev2 Acceptance pass.
