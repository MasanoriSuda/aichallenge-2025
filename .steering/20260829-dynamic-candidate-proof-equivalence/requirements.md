# Requirements: dynamic candidate/proof equivalence

## Objective

Determine why a seven-state ShiftOut QP can satisfy its affine dynamic-obstacle
rows while the unchanged exact swept-footprint certificate rejects the same
trajectory.  Correct the earliest mismatched model, rather than adding solver
tolerances, margins, retries, leases, or fallback paths.

## Frozen inputs

- Reuse the immutable 2026-08-29 ShiftOut failure snapshots.
- Preserve the recorded world/problem fingerprint and exact wall/dynamic/
  terminal-successor proof chain.
- Keep production authority unchanged until the comparison identifies a
  proof-consistent replacement.

## Required classification

1. Separate affine node geometry mismatch from nonlinear integration error and
   inter-stage swept contact.
2. Trace every ordinary behind/side row to its physical footprint source.
3. Compare ordinary scalar rows with footprint support rows at the same frozen
   heading and obstacle prediction.
4. Promote a replacement only if it improves the frozen corpus without
   regressing previously accepted Follow/Cruise/Overtake snapshots.

## Constraints

- No parameter tuning.
- No new grace period, timeout, lease, fallback, or solver-tolerance change.
- Do not weaken exact wall, dynamic, or terminal-successor proof.
- If a new candidate formulation replaces an old one, remove the replaced
  production row construction in the same Slice.
- Preserve ROS 2 and evaluation interfaces.

## Definition of done

- The failure chain is documented from candidate row through exact proof.
- Frozen replay distinguishes node-geometry, inter-stage sweep, integration,
  solver, and physical-infeasibility failures.
- Any production change has focused tests, frozen-corpus evidence, a package
  build, and no parallel legacy production path.
