# Requirements: isolate current-world target binding

## Objective

Add a non-authoritative architecture-comparison arm which preserves the
captured persistent-Mission geometry and binds only the selected current-world
target tube before running the unchanged seven-state SQP and proof pipeline.

## Question answered

The frozen production snapshot omitted the target from the canonical QP.  The
existing stateless-B arm both restores the target and rebuilds the lateral
reference, so its result cannot distinguish target-ownership failure from
persistent-Mission geometry failure.

## Constraints

- Do not change production authority or runtime candidate selection.
- Do not change state/input bounds, references, speed, clearance, solver
  settings, fallback, timeout or lease behavior.
- Rebuild the target stages from the immutable replay world used by exact
  dynamic proof.
- Keep every captured Mission geometry field unchanged.
- Give the counterfactual candidate its own sealed fingerprint.

## Definition of done

- The comparison report contains a distinct persistent-target-bound arm.
- A source with no lower target constraint produces a sealed counterfactual
  candidate without changing its geometry.
- Fingerprint mismatch and incomplete target data fail closed.
- Focused tests, build and package tests pass.
- The production failure snapshot is reclassified with the new arm.
