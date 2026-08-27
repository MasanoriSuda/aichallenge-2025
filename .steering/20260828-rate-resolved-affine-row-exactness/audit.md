# Audit

## Root cause

The solver and artifact validator did not consume precisely the same progress
equation. The solver used a finite-difference approximation of an analytically
linear row; the artifact validator used `progress_delta = v_theta * dt`.

## Classification

`solve succeeds but proof fails: model/certificate mismatch`.

## Non-fix rejected

Increasing the artifact or OSQP tolerance would hide the coefficient mismatch
and would permit the same duplicate-model problem to recur in other rows.

## Implemented correction

The velocity, progress, steering, and response-steering rows are analytically
affine. Their QP rows now use exact coefficients, while the affine offset is
still recomputed from the canonical nonlinear transition at the reference
point. A coefficient-level regression test prevents a finite-difference row
from silently returning.

No solver tolerance, clearance, authority, timeout, lease, grace period, or
fallback was changed.

## Verification

- `make autoware-build`: passed.
- Package CTest: 49/49 passed.
- Aggregate GoogleTest cases: 2008 passed.
- Dynamic run: `output/20260828-025409`.
- The frozen `progress-dynamics-mismatch` rejection did not recur.
- The same run reached `ShiftOut -> Pass`, proving that artifact construction
  advanced beyond the prior rejection.

## Newly isolated downstream defect

During Pass, current target prediction invalidated the retained speed policy,
but a previously certified retained artifact still published positive
acceleration. This is not a recurrence of the affine-row defect. It is a
separate retained longitudinal-certificate/provenance defect and belongs to a
new Slice.
