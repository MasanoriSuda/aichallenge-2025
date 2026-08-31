# Requirements: ShiftOut terminal-Stop authority loss

## Observed phenomenon

In `output/20260831-153609/d1`, Overtake episode 1 did not first fail because
the target became stale.  At decision 1531, while the target remained about
21 m ahead and ShiftOut remained active, the selected production output became
the certified terminal Stop of the ShiftOut source.  From decision 1532 onward
normal forward authority was unavailable, mostly with
`steering-unreachable`.  Speed fell from 3.61 m/s to zero; only later did the
target become irrelevant and the FSM enter Recovery.

## Objective

Determine why a committed, progressing ShiftOut lost forward authority and
could retain only its Stop suffix.

## Constraints

- Freeze production behavior and parameters.
- No timeout, lease, grace, tolerance, clearance or fallback change.
- Treat target stale/lost and Recovery as downstream symptoms.
- Compare the same world snapshot before selecting a repair.

## Definition of Done

- Replay A/B/C/D against the closest pre-loss ShiftOut snapshot.
- Classify physical infeasibility, candidate generation, single-SQP,
  certificate/model mismatch or lifecycle/scheduling.
- Identify the first forward-authority invariant that failed.
- Select at most one structural follow-up supported by evidence.

