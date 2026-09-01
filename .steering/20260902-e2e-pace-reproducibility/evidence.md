# Evidence

## Closed-loop results

| Seed | Acceleration | Result | Laps [s] | Total [s] | Penalty |
| ---: | ---: | --- | --- | ---: | ---: |
| 2026 | 0.8 | pass | 66.94 / 67.34 / 58.27 | 192.55 | 0 |
| 2034 | 0.8 | pass | 65.73 / 65.30 / 59.29 | 190.33 | 0 |
| 2035 | 0.8 | fail | 67.88 | 67.88 | 1 wall |
| 2035 | 0.6 | pass | 104.15 / 89.05 / 99.28 | 292.48 | 0 |

Seed 2034 passed both motion and competition admission.  Seed 2035 at
`0.8 m/s2` entered a wall penalty at race time `120.94 s`, remained penalized
for about `198.6 s`, completed only one lap and timed out.  Its bag contained
`119.73 s` of continuous low speed and `44.73 s` of positive-acceleration
stall.  The exact same seed at `0.6 m/s2` finished 3/3 with zero penalty and
zero stall.

## Root-cause classification

The spatial training evidence has train speed mean `3.128 m/s`, p95
`4.417 m/s` and maximum `4.790 m/s`.  The paired seed 2035 runs measured:

| Acceleration | Mean / max speed [m/s] | Distance [m] | Motion Gate |
| ---: | ---: | ---: | --- |
| 0.8 | 1.391 / 6.515 | 590.81 | fail |
| 0.6 | 3.311 / 4.564 | 1029.92 | pass |

The fixed acceleration parameter owns both transient acceleration and the
drag-limited steady speed.  Raising it to `0.8 m/s2` therefore moved the
lateral policy beyond its evidenced speed envelope instead of only improving
launch and mid-speed response.  This is a longitudinal/lateral distribution
contract defect, not a reason to add a seed-specific escape rule.

## Decision

Reject `0.8 m/s2` as the packaged default and restore the cross-seed-qualified
`0.6 m/s2` baseline.  Preserve the explicit acceleration override and runtime
provenance for diagnostics.  The next bounded hypothesis is a speed governor
that separates stronger transient acceleration from a steady-state limit
within the spatial policy's evidenced speed envelope.
