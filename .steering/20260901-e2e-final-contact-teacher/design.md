# Design

## Controlled variable

```text
d1-d3: tiny_lidar_net + fixed_lidar_brake
d4:    tiny_lidar_net + gap_teacher
world: e2e-final, unchanged
```

Only d4 is changed because the all-production failure produced a stable d2/d4
side-contact pair.  Replacing every vehicle with the teacher would introduce a
second symmetric multi-agent policy and would not isolate whether a single
corrective lateral response can avoid the recorded trap.

## Decision

- d4 passes without contact/stall: collect only the pre-contact and successful
  escape sequence with `lidar_gap_teacher` provenance.
- d4 reaches the same side-contact state: do not retrain; the gap teacher is not
  a valid oracle for this scenario.
- launch or topic contract differs: repair the audit target, not production.
