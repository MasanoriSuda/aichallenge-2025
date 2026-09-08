# Semantic target-time repair

User authorised autonomous execution of the MPCC completion plan on 2026-09-07.
This slice investigates decision 1187 before changing its prediction producer.
Baseline: `a8b968ac2e4a86a28432a52d132774459d355cb5`.

Invariant: obstacle stage k and seven-state input k refer to the same cumulative
semantic time. Publication period and an inactive legacy formulation's period
cannot supply the normal MPCC obstacle clock.

Keep immutable world, model, wall/dynamic/terminal proofs, solver settings,
clearances, costs and authority unchanged. Preserve existing harness and user
artifact changes. A local replay success is not integrated race acceptance.

Rollback: revert this slice's source changes to the baseline, preserving other
working-tree changes. Generated replay evidence is retained.
