# Results: physical diagonal separation candidate F

## Frozen comparison

- Source commit: `43daa303a591fd7575a2532b32c6f2b20c70a6ee`
- Run: `output/20260828-094214`, Domain 1
- Decision: `1566`, Pass wall-refinement solve rejected
- Interaction fingerprint: `7246006054995400977`
- Production authority/configuration changes: none

Candidate F enumerated the same 171 diagonal schedules per homotopy as
candidate E.  The only changed dimension was the separating-row geometry:
normalized axis radii in E versus an asymmetric oriented ego rectangle plus
the exact replay-world peer radius in F.

| Arm | Accepted | Dynamic-proof rejected | Solver rejected |
|---|---:|---:|---:|
| Persistent A | 0 | 0 | 1 |
| Stateless left/right B | 0 | 1 | 1 |
| Rough C | 0 | 0 | 420 |
| Offline D | 0 | 0 | 420 |
| Normalized diagonal left E | 2 | 32 | 137 |
| Normalized diagonal right E | 0 | 0 | 171 |
| **Physical diagonal left F** | **1** | **11** | **159** |
| Physical diagonal right F | 0 | 0 | 171 |

The accepted F bundle was deterministic across repeated comparison runs:

| Start | Full-side | Candidate fingerprint | Terminal progress | Terminal velocity | Minimum wall-bound reserve |
|---:|---:|---:|---:|---:|---:|
| 1 | 3 | `16820872117393555423` | 19.8243 m | 4.62416 m/s | 0.0532911 m |

It passed the unchanged seven-state SQP, exact nonlinear trajectory adapter,
dense swept wall proof, timed all-obstacle dynamic proof and terminal successor
proof.  It is an offline bundle and cannot publish.

Eleven additional left candidates solved but failed the exact dynamic proof.
This is expected evidence that a support plane evaluated at the wall-only
witness heading is an SQP convexification rather than a complete nonlinear
certificate.  These candidates remained non-authoritative.

## Classification

The production repair gate is satisfied for this frozen failure:

> A--D fail, normalized E and physical-model-derived F each form a certified
> diagonal bundle, while the axis-only producer and its partial escape mask do
> not.

The upstream defect is the dynamic-obstacle candidate representation.  The
strict axis disjunction omits a feasible diagonal topology, and
`partial_side_escape` masks the omission with a wall-only witness which the
exact obstacle certificate can reject.

This result does **not** authorize using the affine support row as a final
collision certificate.  Production integration must retain the existing
exact physical proof and fail closed when the proof rejects.

## Production gate

Proceed in a separate Slice with one atomic root repair:

1. derive receding diagonal separating rows from current physical geometry;
2. preserve exact wall/dynamic/successor proof as mandatory authority;
3. delete `partial_side_escape`, its counters and its misleading contract;
4. add the frozen F replay as the deterministic regression;
5. require dynamic Pass-to-Return acceptance before parameter tuning.
