# Results: diagonal obstacle-guidance candidate E

## Frozen evidence

- Production baseline: `ae474be3`
- Run: `output/20260828-094214`, Domain 1
- Decision: `1566`, Pass wall-refinement solve rejected
- Interaction fingerprint: `7246006054995400977`
- Production authority/configuration changes: none

## Same-snapshot comparison

Candidate E enumerated 171 monotone diagonal schedules on each homotopy.  The
candidate reference, seven-state SQP, solver policy, wall model, peer
prediction, exact nonlinear trajectory adapter, exact wall/dynamic proof and
terminal successor proof were unchanged.

| Arm | Result |
|---|---|
| Persistent A | no certified bundle |
| Stateless B | no certified bundle |
| Rough axis-disjunction lattice C | 0 certified / 420 evaluated |
| Offline axis-disjunction continuation D | 0 certified / 420 evaluated |
| Diagonal left E | **2 certified / 171 evaluated** |
| Diagonal right E | 0 certified / 171 evaluated |

The two accepted left bundles were:

| Diagonal start | Full-side stage | Terminal progress | Terminal velocity | Minimum lateral bound reserve |
|---:|---:|---:|---:|---:|
| 0 | 3 | 19.8243 m | 4.59557 m/s | 0.0532999 m |
| 1 | 3 | 19.8243 m | 4.60945 m/s | 0.0532946 m |

Both passed the exact wall proof, dense timed dynamic-obstacle proof and
terminal successor proof.  They are offline bundles and cannot publish.

Thirty-two additional left E candidates solved numerically but were rejected
by the exact dynamic proof.  This confirms that the normalized diagonal row is
only guidance and that the unchanged physical oracle remains necessary.
Rejected first-overlap clearances ranged from approximately `-0.000021 m` to
`-0.006062 m`; none was promoted.

The remaining left schedules and all right schedules were solver-rejected.
This is consistent with the earlier wall evidence: the right homotopy is
blocked by the frozen wall-constrained formulation.

## Classification

The registered exit condition `A/B fail, C succeeds` is generalized here to a
materially different candidate E:

> A--D fail, while a same-world, same-model, same-proof diagonal E succeeds.

Therefore this frozen failure is **candidate/disjunction representation
defect**, not persistent Mission lifecycle failure and not demonstrated
physical infeasibility.  The strict axis-only schedule omits a certified
diagonal topology.  The former partial-escape rule masks that omission by
using a wall-only witness as if it were obstacle separation, which can admit a
collision.

## Production implication

Do not promote the normalized ellipse row directly.  It is not an exact
oriented-footprint model: 32 solved candidates were correctly rejected by the
physical oracle.  The production repair must:

1. generate a tangent/separating half-space from the same physical footprint
   geometry used by the exact certificate;
2. preserve the selected homotopy over the receding horizon;
3. retain exact dense wall/dynamic proof as mandatory authority;
4. delete the wall-only `partial_side_escape` mask in the same Slice;
5. demonstrate the frozen accepted E topology and dynamic Pass/Return before
   parameter tuning.

## Verification so far

- focused CTest: 4/4 targets passed;
- full package CTest: 52/52 targets passed;
- `make autoware-build`: 25 packages built successfully;
- frozen candidate E replay: 342/342 evaluated, 2 certified bundles.
- experiment registry validation: 17/17 entries valid.
