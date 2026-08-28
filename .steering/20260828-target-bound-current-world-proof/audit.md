# Audit

## Observation timeline

| Time | Observation |
|---|---|
| 50.60 s | Pass active, current and predicted footprint sweeps clear |
| 52.50 s | target-bound hold starts from `target[5]` infeasibility |
| 52.80 s | current target sweep changes to unsafe |
| 53.63 s | DP authority released as target-envelope-unsafe |
| 54.31 s | hold ends after `1.80 s / 9.51 m`, without replacement |
| 54.33 s | current body footprints overlap; SafetyBrake owns output |

## Classification

This is a Mission lifecycle / certificate provenance defect. The solver and
the runtime target monitor identified the conflict before contact. Production
continued because a wall-only retained path was mislabeled as a safe execution
prefix and its dynamic proof was optional.

The event does not establish physical infeasibility: the comparison question
is whether a current-world alternative exists. It does establish that the
published A-path was no longer certified and must not retain authority.

## Existing patch interaction

The progress extension and Mission-wide absolute budget did not create the
first target conflict. They amplified it by allowing the same prefix to keep
authority after the short repair interval. Shortening those timers would hide
the provenance defect, so they are not tuned in this Slice.

## Implemented repair

- Target-bound repair now starts from measured lateral position and cannot
  import latest, last-feasible, or aligned warm-start lateral samples.
- Every longitudinally advancing repair prefix requires the current target
  prediction to be valid and sweep-separated. The existing separately
  qualified recoverable-side-contact path is the only exception.
- The target-bound solved-prefix state and its two dedicated budget parameters
  were deleted. The ordinary current-world solved-source handoff remains
  separate and unchanged.

No solver limit, wall clearance, speed policy, lease, timeout, or fallback was
added or tuned.

## Static validation

- `make autoware-build`: 25 packages passed.
- Correct overlay CTest directory:
  `/aichallenge/workspace/build/multi_purpose_mpc_ros`.
- Complete package CTest: 52/52 passed, including 801
  `test_v2x_overtake_core` assertions.
- `git diff --check`: passed.

The similarly named `/aichallenge/build/multi_purpose_mpc_ros` directory is a
stale underlay and still contains the pre-change test binary. It is not valid
evidence for this workspace build.

## Dynamic Gate

Run: `output/20260828-133920/d1/autoware.log`.

Episode 1 entered `Idle -> ShiftOut` and reproduced target-bound infeasibility.
At the decisive near-target event:

| Time | Evidence |
|---|---|
| 21.163 s | current-world freeze admitted with `sweep_clear=1` |
| 21.177 s | prediction changed to `sweep_clear=0` |
| 21.177 s | target-bound hold ended after `0.02 s / 0.12 m` |
| 21.206 s | the next current-world prediction was clear and a new measured-state hold was evaluated |

The frozen failure traveled `1.80 s / 9.51 m` through an unsafe sweep. The
repaired run revoked that authority in one control interval. This satisfies
the Slice objective: old Mission geometry no longer survives a failed current
dynamic certificate.

The first two Overtake episodes did not pass: each remained in ShiftOut and
later entered SafetyBrake. A third independent episode did reach
`ShiftOut -> Pass -> Return`, proving that the repaired path can overtake.
However Return stopped with canonical authority unavailable and completed only
through external Recovery; it was not a clean `Return -> Idle` completion.

These are separate candidate/Mission and Return execution-continuity failures,
not permission to reintroduce the deleted prefix or tune its budget. The next
root-cause investigation compares the failed ShiftOut episodes, the successful
Pass, and the failed Return handoff from this same run.
