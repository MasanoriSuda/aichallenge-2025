# Evidence

## Frozen baseline

The admitted `0.6 m/s2` single-vehicle run had mean forward speed
`3.379 m/s`, maximum speed `4.545 m/s`, zero longitudinal-safety
interventions in `6542` scans and approximately 90-second clear laps.  This
made fixed acceleration, rather than the front brake, the isolated variable.

## Closed-loop A/B

| Run | Acceleration | Environment | Laps [s] | Total [s] | Penalty | Motion Gate |
| --- | ---: | --- | --- | ---: | ---: | --- |
| `20260902-e2e-pace-080` | 0.8 | single | 64.95 / 59.11 / 57.20 | 181.26 | 0 | pass |
| `20260902-e2e-pace-100` | 1.0 | single | 53.01 / 48.89 / 48.46 | 150.36 | 0 | pass |
| `20260902-e2e-pace-100-npc` | 1.0 | NPC | 54.66 / 61.38 / 48.59 | 164.63 | 2 wall | fail |
| `20260902-e2e-pace-080-npc` | 0.8 | NPC | 66.94 / 67.34 / 58.27 | 192.55 | 0 | pass |

All four runs had zero stale scans, zero shadow inference errors and zero
positive-acceleration stall time.  The full-session longitudinal-safety
intervention counts were respectively `0/4328`, `1/3924`, `289/8763` and
`71/9163` controller samples.

The accepted `0.8 m/s2` NPC bag covered `1041.70 m`, had mean forward speed
`5.028 m/s`, maximum speed `7.412 m/s`, and passed both motion and competition
admission.  The `1.0 m/s2` NPC bag also had no stall, but approached obstacles
more closely (`front_min=0.454 m`) and failed the zero-penalty contract.

## Decision

Promote `0.8 m/s2` to the packaged TinyLidarNet launch default.  Keep
`1.0 m/s2` available only through the explicit environment/launch override.
No lateral checkpoint, steering bound, obstacle threshold or recurrent
authority changed in this slice.

## Verification

- `make autoware-build`: 25 packages passed.
- ML workspace pytest: 208 passed.
- TinyLidarNet controller pytest: 61 passed.
- submit/system launch contract pytest: 16 passed.
- `bash -n`, `py_compile`, `docker compose config -q` and
  `git diff --check`: passed.
- Environment-variable-free `make e2e-single` smoke logged
  `tiny_lidar_acceleration: 0.8`, initialized the core with
  `Acceleration: 0.800000`, and completed the first 101 scans with zero stale
  input, safety intervention or inference error.
