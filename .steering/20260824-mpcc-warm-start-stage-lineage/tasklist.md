# Tasklist

- [x] Freeze run `20260824-165722` and baseline `66a76c9`.
- [x] Add failing zero/one/multi-stage lineage tests.
- [x] Return exact stage advance from warm-start compatibility resolution.
- [x] Apply exact advance to primal and every dual stage block.
- [x] Thread and log stage advance in canonical solver contexts.
- [x] Run focused tests, package tests, build and source contract.
- [x] Run bounded dev2 and classify the available Overtake solve failures.
- [x] Review diff for new exceptional branches and commit without run artifacts.

Dynamic qualification note: run `20260824-172712` did not admit an active
ShiftOut before the bounded stop.  It nevertheless produced 25 maximum-
iteration Overtake branch failures, all from cold solves.  This falsifies
warm-stage misalignment as the sole cause of current pre-entry failure, while
the exact-lineage defect remains repaired and deterministically covered.
