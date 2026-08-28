# Results: Dynamic certificate mismatch localization

## Frozen evidence

- Run: `output/20260828-094214`
- Domain: 1
- Snapshot sequence: 1566
- Interaction fingerprint: `7246006054995400977`
- Candidate: stateless positive-side B

The identical sealed snapshot was replayed without changing production
authority, solver settings, clearances, leases, fallbacks or timing.

## Causal localization

The candidate QP solved, but the common exact dynamic proof reported:

- first new overlap at observation time `0.536464 s`;
- first failure inside QP stage 2;
- clearance immediately before that transition: `+0.129684 m` at stage 1;
- clearance at the stage-2 nonlinear node: `-0.143659 m`;
- minimum full physical Frenet disjunction reserve: `-0.209098 m` at stage 2;
- minimum later node clearance: `-0.309220 m` at stage 4.

The affine QP nodes and nonlinear replay nodes agree: their maximum world
position difference is only `0.0000923 m` over the horizon.  The failure is
therefore not caused by the nonlinear rollout diverging from one SQP
linearization, nor only by a collision hidden between two otherwise-clear
nodes.

The solved arm selected positive-side rows from stage 0, with no stay-behind
rows and nine `partial_escape` rows.  Those rows use the obstacle-free wall
witness as a reachable lateral bound even while it is below complete physical
side separation.  They consequently permit a stage node for which neither
the longitudinal-behind nor complete lateral-separation disjunct is true.
The dense proof rejects the resulting collision as designed.

## Root cause and classification

The earliest violated invariant is in candidate generation / obstacle
convexification:

> A partial-side reachability witness was treated as an obstacle-separation
> certificate even though it does not prove either member of the physical
> collision-avoidance disjunction.

The frozen result remains `model_certificate_mismatch` under the registered
exit taxonomy.  More specifically, it localizes to the partial-escape
candidate contract, not persistent Mission lifecycle and not a seven-state
affine/nonlinear state mismatch.

## Next Slice

Do not tune the partial row, clearance or solver.  Evaluate candidate C from
the same world: generate a rough collision-free lateral/progress path whose
nodes already satisfy the complete physical disjunction, then use it only as
the reference/warm start for the unchanged seven-state refinement and exact
proof.  If C succeeds, classify candidate generation as the blocker.  If C
also fails, proceed to offline multi-SQP/nonlinear feasibility D.

