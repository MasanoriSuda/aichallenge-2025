# Requirements: ShiftOut terminal viability boundary

## Observed failure

In `output/20260831-151543/d1`, episode 2 entered ShiftOut with a physically
accepted Mission, then lost normal authority at decision 1838 because its
terminal contingency was unavailable.  Emergency Stop was published before
the FSM reported `actual footprint wall margin violated` at decision 1851.

## Objective

Determine the first upstream boundary that made the committed ShiftOut no
longer recursively stoppable.  Classify the frozen decision-1838 snapshot with
the architecture A/B/C/D comparison before changing production code.

## Constraints

- Do not change Mission resume, lease, grace, timeout or fallback behavior.
- Do not change solver tolerance, wall clearance, vehicle limits or weights.
- Do not treat the downstream Recovery transition as the root cause.
- Preserve production authority while collecting and replaying evidence.

## Definition of Done

- Join the selected/published ShiftOut artifact to the decision-1838 failure.
- Replay persistent, stateless, rough/lattice and offline comparison arms.
- Identify whether the first violated invariant is candidate generation,
  single-SQP, physical infeasibility, proof mismatch or lifecycle/scheduling.
- Select at most one structural follow-up Slice supported by the evidence.
