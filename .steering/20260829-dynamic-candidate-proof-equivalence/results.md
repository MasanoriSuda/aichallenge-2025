# Results: dynamic candidate/proof equivalence

## Root cause confirmed in the ordinary candidate rows

The frozen ShiftOut snapshot records two different obstacle shapes:

- candidate rows used scalar longitudinal/lateral separation values;
- exact acceptance used the asymmetric ego rectangle, heading, safety margin,
  and peer circle.

For the representative dynamic-obstacle failure, the scalar values were
`2.0435 m` longitudinal and `1.55 m` lateral.  The same frozen physical world
requires `2.32 m` behind at zero heading (`1.49 + 0.05 + 0.78`) and `1.555 m`
lateral (`0.725 + 0.05 + 0.78`).  Heading further changes the oriented body
support.  Therefore the QP and its unchanged exact certificate did not define
the same vehicle geometry.

## Replacement

When an immutable physical replay world is present, every ordinary behind,
ahead, and side branch now uses the same oriented rectangle-plus-circle
support function as the exact certificate.  The ahead and behind rows use
different supports because the ego body is asymmetric.  The old scalar
construction is no longer reachable for a physically bound candidate; it is
retained only as compatibility input for callers with no immutable physical
world.

No clearance, tolerance, iteration limit, lease, timeout, fallback, or
production authority was changed.

## Frozen evidence

Snapshot
`20260829-012053/.../000000001612-shiftout-dynamic-obstacle-refinement-solve-rejected`
was replayed before and after the replacement.

- before: persistent-target-bound A2 was rejected by the exact dynamic proof;
- after: the same A2 arm produced an accepted certified bundle;
- the production right `late-physical-diagonal` arm remained accepted.

Snapshot
`20260829-012053/.../000000001675-shiftout-post-refinement-linearization-solve-rejected`
was also replayed after the replacement.

- production left/direct-side was accepted;
- production right/late-physical-diagonal solved its QP but exact dynamic proof
  rejected it;
- that rejected right candidate had positive affine-node clearance
  (`0.183958 m`) but negative nonlinear-node clearance (`-0.0157371 m`) and a
  maximum node position error of about `1.79 m`.

This second result does not invalidate the geometry replacement.  It isolates
the next root cause: the existing post-refinement loop relinearizes vehicle
dynamics only.  Dynamic-obstacle supports remain frozen at the earlier
wall-only heading/witness, so a changed nonlinear trajectory is not followed
by a corresponding obstacle-row rebuild.

## Tests

Affected targets built successfully in the Autoware build container.

The repository-level `make autoware-build` also completed successfully for all
25 packages.  Its only stderr output was the existing setuptools
`setup.py install` deprecation warning.

- `test_mpcc_rate_resolved_dynamic_obstacle`: 20/20 passed
- `test_mpcc_rate_resolved_shadow`: 38/38 passed
- `test_mpcc_architecture_comparison`: 12/12 passed
- `test_mpcc_stateless_maneuver`: 17/17 passed

The new focused test verifies ordinary behind and side rows against the exact
asymmetric physical support, including the target-facing right extent for a
positive-lateral pass.

## Remaining classification

The representative residual failure is not yet an inter-sample-only contact:
its nonlinear stage node is already in collision.  It is also not physical
infeasibility because the opposite production homotopy is certified in the
same frozen world.  The next audit must therefore evaluate proof-consistent
successive convexification before changing margins or candidate counts.
