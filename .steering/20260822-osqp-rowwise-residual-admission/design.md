# OSQP rowwise residual admission hypothesis and rejection

## Initial hypothesis

```text
evaluate_constraint_residuals()
  -> calculates tolerance[row] in each row's own scale
  -> calculates maximum_normalized_constraint_violation

PersistentOsqpSolver::solve()
  -> discards that normalized admission result
  -> calculates one tolerance from maximum scale over the full mixed-unit QP
  -> admits a small-unit row above its own tolerance

Track/Cruise shadow
  -> marks the solution constraint-certified from lateral rows only
  -> later semantic boundary discovers invalid virtual-progress input
```

The initial hypothesis was that the shared adapter claimed more than its per-row diagnostic report
proved.

## Experimental correction

Add a pure `constraint_residuals_satisfied()` predicate over the already constructed report. It
validates dimensions and every finite non-negative `(violation, tolerance)` pair, then requires
`violation <= tolerance` for every row.

`PersistentOsqpSolver::solve()` uses this predicate as the only post-solve residual admission.
The mixed-unit global tolerance calculation is deleted. On rejection it reports the worst row and
uses the existing solver reset path.

## Dynamic falsification

`output/20260822-234326` falsified the shared-boundary correction at the first normal start.

```text
legacy MPC row 166 (first curvature-rate row)
status=solved inaccurate
violation=0.0144887
locally computed tolerance=0.0103059
```

The experimental predicate rejected that result and reset the common workspace.  Subsequent cold
solves reached `maximum iterations reached`; the vehicle stayed at zero speed and accumulated more
than 1,600 solver failures.  Row 166 is the first curvature-rate row for the configured legacy QP,
not a Track/Cruise five-state virtual-progress row.

The local report uses an unscaled physical-row formula, whereas OSQP terminates using its scaled
problem norms.  Requiring that diagnostic formula at the shared solver boundary therefore changes
the accepted semantics of every existing formulation.  The resulting cold-reset cascade is a
larger correctness regression than the three fail-closed canonical fresh-plan misses.

## Selected outcome

Reject and remove the experimental source/test changes.  Keep the strict canonical execution
boundary:

```text
OSQP solution may remain available to its existing formulation consumer
-> canonical execution adapter validates its own semantic rows
-> out-of-contract five-state execution plan is unavailable for that cycle
-> never certify or select that plan
```

Those rare fresh misses are a case for same-formulation retained revalidation, not for weakening
the semantic check or redefining the shared solver's convergence contract.

## Rejected approaches

- Increasing/decreasing OSQP tolerances: changes frequency, not the contract mismatch.
- Normalizing values outside their exact row tolerance: hides a real constraint violation.
- Enforcing every local row tolerance in the shared wrapper: dynamically rejected by the legacy
  start failure above.
- Adding a legacy-only exception or larger multiplier: recreates formulation-specific patching at
  the common boundary.
- Normalizing values outside their exact semantic row tolerance: would hide the three canonical
  fresh-plan misses.

## Authority audit

- No experimental source/test change remains.
- No publisher, selector, Mission, intent, configuration or fallback changed.
- The final authority remains legacy; Track/Cruise canonical remains shadow-only.
- The rejected experiment is retained only as steering evidence so it is not rediscovered as a
  seemingly obvious fix later.
