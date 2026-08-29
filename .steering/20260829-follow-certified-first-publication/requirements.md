# Requirements: Follow certified-first publication

## Objective

Prevent a slow losing Follow escape branch from delaying an already certified
winning homotopy in the single normal worker.

## Evidence

`output/20260829-101711` confirms that persistent side-specific solver contexts
retain their warm starts.  In ordinary Follow windows, average compute fell to
34.920--58.610 ms.  A later window still rose to 217.116 ms with 0.905 s result
age because the population loop solved both sides before publishing one result.

The upper-entry `.steering/ano` log keeps one selected `side` in its main GMPCC
and updates it continuously; asynchronous tactical work does not block the
current control solve.  The relevant architectural property is publication of
the current selected homotopy, not its solver parameters.

## Constraints

- No timeout, lease, grace, fallback, solver tolerance or clearance change.
- Keep one canonical normal authority and one certified-plan Store.
- Persist only target/encounter identity and selected Follow homotopy.
- Evaluate the selected/current source side first and publish immediately when
  its unchanged solver and proofs accept it.
- Evaluate the opposite side only when the preferred side is not certified.
- Do not keep the previous all-certified-branches ranking path in production.

## Definition of done

- Follow population has one explicit homotopy owner keyed to target/intent.
- Candidate ordering is preferred-side first.
- The first certified candidate returns immediately after Store replacement.
- Tests prove a later candidate is not required before publication.
- Dynamic Gate compares compute, result age and authority gaps with
  `output/20260829-101711`.
