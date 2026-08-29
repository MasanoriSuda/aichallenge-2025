# Results: publication-aligned stage bundle

## Implementation

- Retained revalidation now keeps the exact source-time cursor for immutable
  course/progress identity checks and independently chooses the next sealed
  control stage that can own a complete publisher interval.
- The selected command is replayed from the current physical state through the
  unchanged nonlinear continuation, wall, timed dynamic-obstacle, Follow and
  terminal Stop proofs.
- An accepted stage advance is marked as a stateless current-world Bundle, so
  publishing it never records the unmodified source artifact as executed.
- Final-stage exhaustion with no publisher-complete successor stays fail
  closed as `continuation-rejected/invalid-cursor`.
- Aggregate telemetry records source stage, command stage, advance duration,
  per-window count and maximum advance without adding per-cycle log spam.

## Static verification

- `git diff --check`: passed.
- `make autoware-build`: passed, 25 packages.
- full package CTest: 54/54 passed.
- The new failure-first tests cover both an intermediate 15 ms residual that
  advances and a final 5 ms residual that remains rejected.

## Dynamic A/B

Baseline is commit `29562adf`, run `output/20260829-220933`. Candidate is
`output/20260829-223720`, started with `make dev2` and stopped with `make down`.
Counts below use D1 after `V2X race session changed: active=1`.

| Signal | Baseline | Candidate |
|---|---:|---:|
| `continuation=model:invalid-cursor` | 129 | 0 |
| `current=continuation-rejected` | 127 | 0 |
| current-world Bundle decisions | 119 | 96 |
| normal Emergency decisions | 308 | 29 |
| moving normal Emergency decisions | 246 | 26 |

The candidate used 698 stage advances across 69 aggregate D1 telemetry
windows. The maximum observed advance in a window remained below 15 ms,
consistent with skipping only the unusable residual of a roughly 100 ms
solver stage before the 25 ms publication.

D2 also recorded zero `invalid-cursor` and zero continuation rejection. It did
not require a stage advance in this bounded run.

## Classification and residual risk

A failed and B succeeded, so the frozen failure is a publication scheduling /
lifecycle connection defect. A new candidate generator or offline nonlinear
solve was not required. No physical proof or parameter was relaxed.

The run still contains 26 moving Emergency decisions and failed Overtake
episodes. Their logged causes are distinct (wall/entry feasibility and normal
authority availability); this Slice does not claim Overtake completion or the
integration Gate. They must be frozen as the next failure family rather than
mixed into this repair.
