# Design: physical diagonal separation audit

## Root-cause hypothesis

Candidate E proved that the frozen world contains a certified diagonal pass
topology which the strict behind/side/ahead axis disjunction cannot express.
However, its normalized elliptical row was only guidance: 32 solved candidates
penetrated the exact rectangle-plus-circle body model.  Promoting that row
would replace a visible failure with a certificate mismatch.

## Physical support half-space

At a predicted stage, let the unit course-frame normal pointing from ego
towards the target be

```text
n = [cos(alpha), -q sin(alpha)]
```

where `q` is the selected ego pass side.  Let the wall-only SQP witness supply
the ego heading offset used for this convexification.  Rotate `n` into the ego
body frame and evaluate the support function of the exact asymmetric
rectangle expanded by the configured footprint margin.  Add the opponent
circle radius:

```text
h(n) = support(expanded ego rectangle, n) + opponent_radius
```

The affine stage row is

```text
n dot (target_center - ego_center) >= h(n)
```

Using physical progress `p = theta + e_lag` and lateral state `d`, this becomes

```text
-cos(alpha) * p + q sin(alpha) * d
  >= h(n) - cos(alpha) * p_target
             + q sin(alpha) * d_target
```

At `alpha=0`, this is a front-body stay-behind plane.  At `alpha=pi/2`,
it is the selected-side body plane.  Intermediate rows continuously rotate the
separating normal while remaining tied to the physical body model.

This is still an SQP linearization: the support is evaluated at the witness
heading while heading remains a decision state.  Therefore the unchanged
nonlinear proof remains the authority oracle; a QP solve alone is not an
acceptance result.

## Candidate F comparison

- Reuse the bounded candidate-E diagonal schedules on left and right.
- Rebuild physical geometry only from `Snapshot::replay_world`.
- Seal the physical-guidance contract into each candidate fingerprint.
- Run the same seven-state SQP, wall refinement, nonlinear adapter and proofs.
- Record solver rejection separately from wall, dynamic and successor proof
  rejection.

## Production decision

- F certified: implement the same physical support producer in the live
  receding path, then delete `partial_side_escape` in the same production
  Slice.
- F only solves but proof rejects: model/certificate mismatch remains; do not
  promote.
- F does not solve: candidate representation is still incomplete or the
  chosen linearization is inadequate; pause and compare topology generation
  with current T-MPC++/guidance-planner literature and the recorded upper-rank
  GMPCC behavior before adding rules.
