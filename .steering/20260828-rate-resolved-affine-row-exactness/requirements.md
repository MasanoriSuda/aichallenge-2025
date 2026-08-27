# Requirements

## Objective

Remove the model/artifact mismatch exposed by the dynamic Gate as
`progress-dynamics-mismatch` without changing solver tolerances.

## Evidence

- Run: `output/20260828-024009`
- Snapshot:
  `d1/mpcc_architecture_snapshots/000000003777-shiftout-physical-proof-artifact-construction-rejected/snapshot.yaml`
- The final progress defect exceeded the artifact residual envelope by only
  numerical-Jacobian error, although progress propagation is analytically
  affine.

## Constraints

- No solver tolerance, clearance, lease, timeout, or authority change.
- Keep the shared nonlinear transition as the canonical state propagation.
- Replace numerical derivatives only for rows with exact closed-form affine
  dynamics.
- Add coefficient-level regression tests.

## Definition of done

- Velocity, virtual progress, command steering, and yaw-response steering rows
  use exact analytic Jacobians.
- The affine tangent still passes through the nonlinear reference transition.
- Focused and package tests pass.
- The captured `progress-dynamics-mismatch` signature does not recur in the
  next dynamic Gate.
