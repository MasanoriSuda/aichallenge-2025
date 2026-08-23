# Design (rejected)

## Root cause

```text
mixed-unit five-state QP
-> OSQP global convergence accepts a locally out-of-bound raw primal
-> the semantic boundary rejects the raw field
-> retained world proof is often unavailable in one-car NoData state
-> one-cycle Emergency brake
```

The previous normalizer also projects state and input fields independently
when they are only slightly outside a bound. That makes the published
execution artifact differ from the dynamics equality that produced the
certificate. Increasing its accepted adjustment would therefore hide the
symptom and worsen the proof boundary.

## Canonical restoration

The new pure restoration step consumes:

- raw five-state primal;
- actual QP constraint matrix and lower/upper bounds;
- the per-stage `ExtendedLinearization` objects used for QP assembly;
- the horizon length.

It performs one deterministic projection:

1. Copy stage zero from the measured hard equality.
2. Project acceleration and virtual-progress speed into their input boxes.
3. Project curvature sequentially into the intersection of its input box and
   the reachable curvature-rate interval from the previous restored stage.
4. Roll `x[k+1] = A[k] x[k] + B[k] u[k] - c[k]` using the same equality
   offsets supplied to the QP.
5. Reject if any rolled state leaves its declared box.
6. Evaluate the complete QP matrix against all bounds with a strict local
   numerical tolerance.

The output is either a wholly recertifiable primal or a typed rejection. No
partially repaired candidate is returned.

## Authority and provenance

The raw OSQP primal/dual remain stored by `solve_extended_progress_problem()`
before restoration and continue to seed the next warm solve. The restored
primal is used only below the canonical execution boundary:

```text
raw OSQP result
  |-- raw residual/boundary telemetry
  |-- raw warm-start storage
  `-- deterministic restoration
        -> complete QP-row proof
        -> physical wall/world proof
        -> canonical plan store
        -> publisher authority
```

This introduces no competing authority and no compatibility branch.

## Failure-first cases

- stage-0 curvature outside its box and rate interval;
- acceleration above its input box, with stage-1 velocity reconstructed;
- negative virtual-progress speed, with progress/lag reconstructed;
- predicted velocity outside its box because the raw state is inconsistent;
- projected curvature whose only feasible value makes a later lateral state
  leave its hard corridor (must reject);
- empty curvature box/rate intersection (must reject);
- malformed matrix/linearization provenance (must reject).

## Deletion boundary

Once production uses restoration, independently clamped normalized primal
must no longer be executable. The existing boundary scan may remain only as
raw diagnostic provenance; it cannot own the published candidate.

## Dynamic falsification

The design assumed the raw horizon was dynamically consistent except for a
small number of actuator/state boundary values. The production run disproved
that assumption:

```text
raw primal
-> exact control projection and dynamics rollout
-> 120--145 fields changed per sampled cycle
-> maximum change 0.58--3.13
-> restoration reject or a substantially different physical trajectory
-> wall certificate reject / Emergency / Reverse
```

The correct upstream problem is therefore not a missing projection at the
publisher boundary. The five-state QP is reaching its iteration limit with a
mixed-unit primal whose dynamics/state feasibility is too weak to grant
authority. Reconstructing a different horizon downstream hides that defect.

The next Slice must inspect formulation scaling and row-wise residuals at the
solver boundary. It must not add another post-solve repair, widen tolerance,
or tune racing parameters.
