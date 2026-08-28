# Design: diagonal obstacle-guidance comparison

## Earliest suspected violation

The current convex obstacle producer represents every predicted stage as one
axis-aligned disjunct:

1. effective progress completely behind the peer;
2. lateral center separation completely on the selected side; or
3. effective progress completely ahead of the peer.

The frozen left candidate has a wall-only witness but bounded continuation
cannot move directly from complete-behind to complete-side.  The production
partial-escape branch avoids that empty intersection by placing the lateral
row on the wall-only witness.  The same frozen B-left solution then violates
the exact dynamic proof.  Therefore strict axis disjuncts are a candidate
representation defect hypothesis, while partial escape is its unsafe mask.

## Shadow-only candidate E

For a selected side `q in {-1,+1}`, predicted peer progress `p_t`, lateral
position `d_t`, longitudinal separation `L`, lateral separation `S`, and ego
physical progress `p = theta + e_lag`, add the normalized supporting row

```text
cos(alpha) * (p_t - p) / L
  + sin(alpha) * q * (d - d_t) / S >= 1
```

`alpha=0` is exactly the existing stay-behind row and `alpha=pi/2` is exactly
the existing selected-side row.  Intermediate alpha values represent a
diagonal guidance topology with one affine row containing both state axes.

This normalized boundary is only a candidate convexification.  It is not a
replacement for the exact oriented-footprint certificate.  Every solved E
candidate must pass the unchanged nonlinear wall and dynamic proofs.

## Bounded enumeration

For each left/right homotopy, enumerate:

- first diagonal stage `0 .. N-2`;
- first full-side stage `first+1 .. N-1`.

Stages before the diagonal start use complete stay-behind.  The selected
support angle interpolates monotonically to complete-side, which is then held
for the remaining horizon.  Candidate reference, state/input bounds, model,
solver and proofs remain stateless-B-identical so the comparison isolates the
constraint representation.

## Classification

- E exact bundle accepted: A--D omitted a feasible diagonal topology;
  candidate/disjunction representation is the upstream defect.
- E numerically solves but exact proof rejects: obstacle model/certificate
  mismatch remains.
- E solver rejects: physical feasibility remains `Unknown`; the result is not
  an infeasibility certificate.

## Production impact

None.  The extra request fields and combined row semantics are architecture
comparison inputs only.  Live snapshots retain default values and take the
unchanged production branch.
