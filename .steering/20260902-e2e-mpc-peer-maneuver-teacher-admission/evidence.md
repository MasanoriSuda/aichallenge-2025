# Evidence

## Frozen run

- Output: `output/20260902-e2e-mpc-peer-maneuver-teacher`
- Command: `make e2e-peer-audit-mpc`
- Runtime identity: every domain reports `control_method: mpc`.
- Required result: three laps, zero penalty, zero sustained stall.
- Shutdown was completed before analysis; all bags and result JSON files were
  finalized.

No TinyLidar checkpoint, teacher-mode, residual, recurrent or E2E authority
override was present in this experiment.

## Race result

| Domain | Laps | Finished | Penalty | Motion distance | Longest low speed | Verdict |
|---|---:|---|---|---:|---:|---|
| d1 | 0/3 | no | crash, 19.43 s | 249.76 m | 8.33 s | reject |
| d2 | 0/3 | no | wall, 6.09 s | 241.15 m | 5.04 s | reject |
| d3 | 1/3 | no | wall, 2.05 s | 403.76 m | 3.96 s | reject |

The per-bag motion analyzer's local stall verdict passes d1 and d3, but that is
only a motion continuity check.  It cannot override the run-level admission
contract: all three vehicles failed to Finish and all three incurred a penalty.
d2 also failed the positive-acceleration stall threshold at 5.04 seconds.

## Causal evidence

The failure was not caused by the E2E student or by missing input topics.

- d1 entered `Idle -> ShiftOut`, then reported `solver_unsafe` and left
  `ShiftOut -> Recovery` because the Pass-entry physical gate had no valid
  current-side prefix.  The episode ended as `recovery stalled` and later
  produced repeated `retained-proof-unavailable` emergency authority.
- d2 entered ShiftOut twice.  The second episode was rejected with
  `actual footprint wall margin violated`, followed by
  `ShiftOut -> Recovery` and `recovery stalled`.  The result records a wall
  penalty.
- d3 did not enter an Overtake phase in the captured log.  Normal Cruise lost
  canonical authority through `retained-proof-unavailable`, entered
  solver-unsafe stuck recovery, and later received a wall penalty.

Occurrence counts in the finalized logs further show that the failures are not
single harmless transients:

| Domain | `canonical-stop-emergency` | `retained-proof-unavailable` | `stuck-recovery-arbitration` |
|---|---:|---:|---:|
| d1 | 43 | 69 | 9 |
| d2 | 27 | 51 | 10 |
| d3 | 19 | 42 | 7 |

## Decision

Reject the current mixed-peer MPC run as a training source.  In particular:

- do not extract its control commands;
- do not append it to the MPC imitation dataset;
- do not tune MPC/MPCC thresholds in this E2E Slice merely to admit it;
- do not add the MPC controller as an E2E runtime fallback.

The existing single-car MPC demonstrations remain valid for lane-following
imitation.  This result only rejects the stronger claim that the current
MPC/MPCC stack already supplies reliable complete-maneuver labels in this
three-peer world.

## Consequence for the next Slice

The two immediately available teachers are now both rejected for the missing
mixed-peer capability:

1. `speed_committed_teacher` has no time-indexed swept-footprint/terminal escape
   certificate; and
2. current MPC/MPCC does represent a complete trajectory, but does not reliably
   Finish this world without penalties or authority loss.

The next step must therefore compare bounded alternatives before changing
production: a successful human/privileged demonstration source, or an offline
short-horizon maneuver candidate/certificate that is evaluated against the
same frozen peer failures.  Re-labeling either failed run would hide the root
cause rather than teach an escape maneuver.
