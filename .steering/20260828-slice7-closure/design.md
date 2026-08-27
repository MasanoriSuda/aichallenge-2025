# Design

## Decision boundary

Slice 7 tunes only a proven canonical architecture.  It does not use a
parameter change to hide an authority discontinuity, missing terminal
successor, model/certificate mismatch, or wall infeasibility.

The accepted comparison baseline is commit `b273d56d`, production horizon
`N=20`, and production solve submission at the 40 Hz control cadence.  Dynamic
run `output/20260828-044759` contains a complete
`Idle -> ShiftOut -> Pass -> Return -> Idle` episode without Overtake Recovery
or actual-footprint wall violation.

## Tested families

1. Reduce the canonical horizon from 20 to 16 stages.
2. Reduce it conservatively from 20 to 18 stages.
3. Keep 20 stages but reduce production solve submission from 40 to 20 Hz,
   while retaining 40 Hz publication/current-world validation.

Each candidate reduced some measured work.  Each also lost reproducible
terminal successor or authority continuity in a dynamic Overtake episode.
Therefore no parameter delta is accepted.

## Closure semantics

“Slice 7 complete” means the bounded tuning campaign is complete with no
accepted change.  It does not mean race-production quality is complete.  The
baseline still has callback tails, maximum-iteration events, and short
canonical Emergency intervals.  Those are typed architecture/integration
backlog and must not be addressed by silently shortening the proof or lowering
the solve cadence.
