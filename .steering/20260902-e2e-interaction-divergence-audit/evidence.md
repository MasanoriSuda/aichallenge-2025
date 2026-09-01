# Evidence

## Frozen inputs

- Run: `output/20260902-e2e-final-speed-aware-safety`
- Candidate SHA256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- Candidate uses embedded base steering and the production `+/-1.2 rad` bound.
- Report: `output/20260902-e2e-final-speed-aware-safety/interaction-divergence.json`

No runtime authority, checkpoint, or parameter was changed by this audit.

## Same-world comparison

| domain | outcome | comparison window | side-hazard samples / episodes | within-episode side flips | teacher deficit | toward obstacle |
|---|---|---|---:|---:|---:|---:|
| d1 | stalled at 82.85 s, recovered | ten seconds before stall | 0 / 0 | 0 | N/A | N/A |
| d2 | stalled at 62.78 s, no recovery | ten seconds before stall | 42 / 2 | 1 | 76.2% | 40.5% |
| d3 | clean four-lap run | first 60 seconds after motion | 150 / 6 | 2 | 86.0% | 56.7% |
| d4 | clean four-lap run | first 60 seconds after motion | 45 / 8 | 0 | 77.8% | 13.3% |

`teacher deficit` means that the diagnostic pre-contact teacher requested at
least 0.05 rad more steering in its physical escape direction.  It is not a
ground-truth error label.  Side flips are counted only inside one contiguous
hazard episode; false samples and recorder gaps cannot join two episodes.

## Classification

- d2 does contain sustained tight-side evidence, falling speed and one genuine
  escape-side reversal before its unrecovered stall.
- The teacher deficit is not failure-specific.  It is similarly large in clean
  d3/d4, and clean d3 also contains two within-episode side flips.
- d1 has no teacher side-hazard event in its immediate causal window, so the two
  stalled domains do not share a single threshold-level lateral explanation.
- Therefore neither stronger imitation of the current pre-contact teacher nor
  a blanket side-commit timeout is supported by this run.

The next closed-loop experiment must first produce an executed, run-admitted
dynamic-interaction teacher trajectory.  Only its successful pre-contact
samples may become hard supervision.  Production v11 remains frozen.

## Upper-run reference boundary

`.steering/ano/rosbag2_autoware.mcap` is a 324.67-second GMPCC reference with
`control_cmd`, odometry, acceleration, V2X and clock.  It does **not** contain
2D LiDAR or camera topics.  Its log proves continuous opponent-aware control,
`N=20`, `dt=0.12`, a 2.4-second horizon and asynchronous two-branch solves,
but the bag cannot directly supply LiDAR-to-steering E2E training pairs.  Use it
as a behavioral/pace reference, not as an E2E label source.
