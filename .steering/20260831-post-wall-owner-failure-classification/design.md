# Design

## Failure families

1. Progress-aligned wall refinement solve rejection.
2. Coupled wall/dynamic refinement solve rejection.
3. Dynamic-obstacle refinement solve rejection during Cruise/ShiftOut.
4. Terminal contingency unavailable after an otherwise physical candidate.

## Comparison

For representative frozen snapshots evaluate:

- A: persistent source candidate and seven-state SQP.
- B: stateless left/right ManeuverBundle and the same seven-state SQP.
- C: rough lattice candidate with seven-state refinement.
- D: bounded offline multi-SQP/nonlinear feasibility evidence.

Classify using the architecture escape-hatch contract.  A target-free Cruise
may make B/C unavailable; this is not evidence for physical infeasibility.
When the comparison harness cannot express an arm, record the evidence gap
instead of inferring a result.

## Exit

Choose exactly one next production Slice from the highest-frequency failure
whose root cause is supported by both frozen replay and live logs.  All other
families remain frozen follow-up work.

## Result

The architecture snapshots are not all production-authority failures.  The
producer and live consumer must be identified before applying the A/B/C/D
classification:

- The three `ShiftOut` snapshots were produced by the pre-entry execution
  shadow.  No `ShiftOut`, `Pass` or `Return` canonical production command was
  published in any of d1, d2 or d3 during the candidate run.
- Cruise snapshot 2684 reproduced `A fails, B succeeds`, but the live log
  records `store=accepted/adopted_side=1`.  The normal sibling path already
  localized that primary branch failure; it did not become global authority
  loss.
- Cruise snapshot 1128 had no certified Bundle from persistent, stateless or
  production direct-side arms.  Exact dynamic proof rejected all direct arms
  for a new overlap with d2.  The terminal-contingency symptom must not be
  repaired by weakening the Stop proof.

The next production Slice therefore does not modify branch adoption, wall
clearance or Stop proof.  It traces why tactical `Follow -> Overtake` proposals
never become canonical `ShiftOut` production intent in this run.
