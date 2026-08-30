# Requirements

## Objective

Determine whether the accepted observation-only Stop lattice artifact can
join the exact current-world seven-state authority boundary when ordinary
ShiftOut/Pass authority first becomes unavailable.

## Frozen evidence

Run `output/20260830-233212` lost ordinary Pass authority at decision 1494.
The previously published ShiftOut continuation was rejected, external Stop
became the retained authority on the next cycle, and the vehicle stopped
before Return.  The Stop lattice had produced earlier accepted observations,
while the newest source was still running and was later superseded.

## Constraints

- Do not change production authority, command selection or Store contents.
- Do not add a Mission rule, lease, grace, timeout, fallback or retry.
- Do not change solver tolerance, wall clearance, candidate population or
  candidate ordering.
- Join must use the existing current-world retained evaluator and unchanged
  exact wall, peer and terminal contracts.
- The observed lattice plan may be retained only as diagnostic evidence and
  must be cleared when its published Overtake source lifecycle ends.

## Exit classification

- no accepted plan at first loss: producer scheduling/freshness defect;
- accepted plan exists and current-world join succeeds: missing production
  source-replacement edge;
- join rejects on intent/identity: provenance or lifecycle defect;
- join solves but physical proof rejects: model/certificate mismatch;
- join rejects at solver/current-state continuation: stale candidate or
  candidate-generation defect, to be replayed from the same snapshot.

## Definition of Done

- Failure-first tests fix the observation-only ownership boundary.
- Bounded telemetry distinguishes missing plan, attempted join, accepted join
  and typed current-world rejection.
- Source audit proves the observer cannot write the Store or publisher.
- Build and package tests pass.
- A `make dev2` run reaches a real ShiftOut/Pass authority-loss boundary and
  yields one of the exit classifications.
