# Evidence

## Implementation and tests

- Pure module: `lib/maneuver_rollout.py`
- Replay: `audit_swept_maneuver_certificate.py`
- Physical constants:
  - wheelbase `1.087 m`;
  - front/rear extent `1.554/0.510 m` from rear-axle base link;
  - vehicle half width `0.725 m`;
  - LiDAR longitudinal offset `1.65 m`;
  - clearance `0.15 m`;
  - Stop suffix deceleration `1.0 m/s2`.
- Candidate steering offsets:
  `[-0.32, -0.24, -0.16, 0.00, 0.16, 0.24, 0.32] rad`.
- Focused tests: 9 passed.

The candidate bank is evaluated around the actually published steering.  Each
candidate contains a 0.45-second shift, a 0.45-second counter-shift and a
zero-steer full-stop suffix.  Feasibility requires every sampled rectangular
footprint to retain the frozen clearance.

## Frozen replay

Report:
`output/20260902-e2e-peer-speed-committed-teacher/swept-maneuver-certificate-audit.json`

| Case | Evaluated | Any candidate | No candidate | Selected unavailable / opposite available | Teacher forward without selected certificate |
|---|---:|---:|---:|---:|---:|
| success train | 160 | 52.50% | 47.50% | 9.38% | 58.13% |
| success validation | 160 | 54.38% | 45.63% | 11.88% | 58.13% |
| failed peer, last 20 s | 85 | 48.24% | 51.76% | 8.24% | 52.94% |

The failed case is not enriched for a wrong selected side.  Its 8.24% rate is
lower than both successful runs.  The static certificate also rejects roughly
half of relevant successful frames, so it is too conservative and incomplete
to serve as a label admission rule.

## Root-cause classification

`current-scan-swept-rollout-does-not-isolate-failure`

Adding vehicle dimensions and a Stop suffix to a single scan is necessary but
not sufficient.  It cannot tell a moving peer from a wall, observe an occluded
future corridor or account for the peer changing side during execution.  The
result independently confirms the previous audit: the missing information is
temporal/dynamic trajectory evidence, not another polar-gap or clearance
threshold.

## Decision

- Do not generate labels from this candidate bank.
- Do not connect it to runtime authority.
- Do not tune offsets, clearance or braking merely to separate these three
  bags.
- Keep the production checkpoint and spatial adapter unchanged.

The next architecture experiment must first show that time-indexed peer
occupancy or a successful privileged trajectory source discriminates the same
frozen failure.  Only then may it become a teacher for the ML lateral policy.
