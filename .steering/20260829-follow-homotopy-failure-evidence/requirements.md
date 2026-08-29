# Requirements: Follow homotopy failure evidence

## Frozen evidence

- Baseline: `141ba49b`
- Run: `output/20260829-145849`, Domain 1
- Failure decision: 1130, worker sequence 531

The frozen positive-side Follow snapshot fails both as production A and as a
fresh same-candidate replay.  A freshly rebuilt negative-side candidate from
the same world succeeds.  Production evaluates both sides, but the snapshot
recorder currently deduplicates by intent/stage/outcome and can retain only
one side.  It therefore cannot establish whether the production negative-side
failure is numerical-context state or a different candidate problem.

## Objective

Preserve one replay-ready failure snapshot per physical Follow homotopy so the
same candidate can be compared under production and fresh solver contexts.

## Constraints

- Observation-only; do not change command authority or candidate selection.
- Do not change solver settings, clearances, timing, fallback or lifecycle.
- Keep capture bounded to one artifact per intent/side/stage/outcome/process.
- Do not emit every current-world fingerprint.

## Definition of done

- Opposite Follow sides no longer suppress each other's first failure.
- Repeated failures on the same side remain deduplicated.
- Focused tests and full package build pass.
- A bounded `make dev2` run produces side-distinct evidence when both sides
  fail, allowing exact same-candidate fresh replay.
