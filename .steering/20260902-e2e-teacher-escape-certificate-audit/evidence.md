# Evidence

Report (ignored runtime artifact):
`output/20260902-e2e-peer-speed-committed-teacher/teacher-escape-contract-audit.json`.

The audit sequentially replayed the exact packaged raw checkpoint and
stateful teacher over two certified successful bags and the frozen failed
mixed-peer bag.  Wheel speed was synchronized by latest-preceding sample with
the executed 100 ms freshness contract.

## Static data-flow finding

`LidarSpeedCommittedTeacher.dynamic_distances()` computes a physical stopping
distance, but `decide()` uses it only to enlarge the gap proposal's trigger
distance.  Published acceleration continues to come from the historical fixed
`3.0 m` slow and `1.5 m` stop envelopes, except for bilateral pinch, no-gap,
late-side-switch and the same static stop condition.

The selected gap is an instantaneous angular segment.  Its decision contains
no trajectory, swept footprint, time-indexed opponent occupancy or terminal
successor proof.  Thus `side-maintained` proves temporal homotopy identity, not
physical escape viability.

Coarse runtime telemetry already refutes a naive dynamic-stop brake rule.  The
failed run reports `front=4.59 m`, `required_stop=21.27 m` immediately before
the trap, but certified successful runs repeatedly report similar states such
as `front=5.53 m`, `required_stop=21.04 m` and continue to Finish.  Exact replay
confirmed the same result.

## Exact comparison

| Case | Outcome | Admitted scans | Inside dynamic stop envelope | Committed gap + forward | Longest such episode |
|---|---|---:|---:|---:|---:|
| final peer d3 | certified success | 4,371 | 80.94% | 25.49% | 3.35 s |
| independent final peer d3 | certified success | 4,463 | 75.35% | 22.56% | 3.92 s |
| mixed MPC peer d3 | stall failure | 4,496 | 84.59% | 12.30% | 2.37 s |

The failed run's final 20 seconds were inside the dynamic stop envelope for
98.98% of admitted samples and contained 71 committed-gap forward samples.
However, both successful cases contain a larger fraction and longer episodes
of that same predicate.  Braking at every dynamic-envelope entry would reject
normal successful cornering and interaction states; the predicate does not
discriminate failure.

Replay acceleration matches the published command at p95 to floating-point
precision in all cases.  The failed case has one 1.8 m/s2 maximum mismatch
around an input-freshness boundary; its mean error is only 0.00254 m/s2 and it
does not explain the sustained trajectory failure.

## Root-cause classification

`instantaneous-gap-lacks-escape-certificate`.

The teacher currently proves:

- an instantaneous angular opening;
- a temporally committed side identity;
- a speed-dependent stopping-distance observation.

It does not prove:

- a time-indexed trajectory;
- swept vehicle-footprint clearance;
- dynamic-obstacle occupancy over the horizon;
- a viable terminal successor after the lateral escape.

The code therefore equates `side-maintained` with permission to continue, even
though that state only says the proposed angular sign remained the same.  In
the mixed-peer failure, an instantaneous opening disappears before a complete
vehicle trajectory can clear it; the fixed static stop envelope then brakes
only after the vehicle is already physically trapped.

## Decision

Do not tune the existing teacher's stop, slow, trigger, side or speed values.
Do not use the failed teacher run as a hard-label source.  A further
`speed_committed_teacher` patch would add another heuristic without creating
the missing proof.

The next candidate-generation Slice must compare complete short-horizon
maneuvers before authority: at minimum current-side and opposite-side
trajectory candidates with vehicle-body clearance and a terminal continuation
or Stop suffix.  The existing teacher may remain as a historical diagnostic,
but it is not the source of corrective labels for the frozen mixed-peer case.

## Verification

- focused audit tests: `3 passed`;
- complete TinyLidarNet ML suite: `233 passed`;
- Python byte-code compilation: pass;
- `git diff --check`: pass;
- production launch, runtime teacher, checkpoints and authority: unchanged.
