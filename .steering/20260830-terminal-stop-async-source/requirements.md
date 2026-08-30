# Requirements

## Evidence boundary

- Baseline: `7396a577 audit(mpcc): prove bounded stop control lattice`
- Frozen production authority, publisher, wall clearance, solver tolerance and
  Mission timing.
- The accepted lattice is evidence of a candidate-generation defect, not
  permission to add another synchronous fallback.

## Objective

Prepare one bounded terminal-Stop candidate source that can be evaluated from
the same immutable epoch as a normal seven-state solution.  The first Slice is
observation-only and must establish a reusable candidate boundary before any
production authority changes.

## Constraints

- Extract maximum-braking rebasing and steering-lattice generation from the
  architecture comparison into a production-neutral component.
- Keep architecture audit results bit-for-bit equivalent at the contract
  boundary.
- Do not write the certified Store, publish a command or add a fallback.
- Do not run a Stop SQP from the 40 Hz control callback.
- A future live shadow must execute after normal candidate certification on a
  separate latest-only worker and carry the exact source identity.
- Promotion must replace fixed/path-feedback Stop generation and remove its
  production call path in the same Slice; permanent dual ownership is not
  allowed.

## Exit gate

- reusable deterministic candidate builder with focused tests;
- existing architecture comparison continues to certify decisions 4017 and
  4489;
- full package tests pass;
- documented async/Store/publisher integration seam and Return limitation.
