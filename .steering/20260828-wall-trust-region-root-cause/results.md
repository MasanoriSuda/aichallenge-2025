# Results

## Observed failure

Decision 2473 entered canonical ShiftOut with target `d2`, side `-1`, and the
seven-state formulation. The broad/relinearized problem produced a provisional
trajectory, but the subsequent physical wall-refined affine QP did not solve.

## Architecture comparison

The immutable current world was replayed through the existing architecture
comparator.

- persistent A: rejected at wall refinement;
- target-rebound A2: rejected at the same wall refinement;
- stateless B left/right: both rejected at the same wall refinement;
- rough C and physical-diagonal candidates: rejected at the same wall
  refinement;
- production-bounded G left/right: both candidates per side were rejected at
  the same wall refinement.

Changing Mission persistence, side, dynamic-obstacle disjunction schedule, or
lateral reference did not reach exact wall/dynamic proof. Adding an active
Overtake version of the Follow escape population would therefore only repeat
the same failure and was not implemented.

## Exact affine feasibility

Independent SciPy/HiGHS LP reports the recorded final QP as infeasible. This
rejects an OSQP-iteration or warm-start explanation for this snapshot.

- variables: 207;
- equality rows: 147;
- dynamic-obstacle rows: 0 at the failed stage;
- minimum common all-row slack: `0.046016819`;
- no single scalar bound removal restores feasibility.

Removing only the refinement-owned heading state boxes restores feasibility.
Removing dynamics or every state box also restores feasibility. Removing only
explicit progress/swept wall rows, steering-prefix rows, lag boxes, lateral
boxes, progress boxes, or any input family does not.

The elastic witness needs `0.046016819` of dimensionless common row relief and
activates adjacent heading, lag, progress, and steering-prefix constraints.
When only the heading state boxes are relaxed, at least `0.123193468 rad` is
required. When all refinement-owned pose boxes are relaxed with one normalized
slack, the minimum is `0.098783930`. This is a coupled pose/actuation
contradiction, not an isolated wall-margin threshold.

## Root cause

The wall proof is added after a broad solution which is not required to carry
the oriented vehicle footprint through a stagewise physical wall corridor.
Physical refinement then:

1. chooses a clear lateral interval at the provisional progress/lag/heading;
2. freezes progress and lag into 0.05 m buckets;
3. freezes heading into a 0.025 rad bucket;
4. adds progress-aligned and swept lateral wall rows;
5. asks one affine QP to move into that interval.

For decision 2473, the necessary lateral correction is not reachable through
the affine dynamics while remaining inside those pose buckets and the
serialized steering prefix. The refinement is described as a trust region
around the provisional trajectory, but its combined physical constraints do
not contain an affine-feasible continuation of that trajectory.

The failure is therefore upstream of persistent Mission lifecycle and
downstream of the bounded left/right candidate choice. The current evidence
classifies it as a **single-SQP / wall-refinement construction defect**.
Physical infeasibility is not established: an alternate nonlinear tangent or
wall-feasible seed has not yet been solved.

## Existing patch relationship

- More side retries cannot help because both sides share the same refinement
  failure.
- More OSQP iterations cannot make an empty affine feasible set non-empty.
- Relaxing clearance would hide the mismatch rather than repair the tangent.
- DynamicMissionWait and Recovery observe the downstream absence of a
  certificate; they do not cause this QP to be empty.
- The post-refinement multi-SQP loop starts only after the first wall-refined
  QP succeeds, so it cannot repair this earlier failure.

## Upper-rank comparison

The stored upper-rank log in `.steering/ano` shows a continuously running
GMPCC with bounded asynchronous tactical branches. The
[ETH Zurich MPCC reference implementation](https://github.com/alexliniger/MPCC)
uses track half-spaces inside the time-varying QP and a dynamic-programming
obstacle corridor before the QP solve. It does not add an oriented-wall pose
bucket after an unconstrained trajectory has already been selected.

The current [acados SQP examples](https://github.com/acados/acados/tree/main/examples/acados_python/convex_problem_globalization_needed)
also distinguish globalization/feasibility restoration from merely increasing
QP iterations. This does not prescribe an acados dependency here, but it
supports testing a new feasible tangent rather than tuning OSQP or wall margin.
Neither external source proves the upper-rank implementation, so the repository
snapshot and exact LP remain the evidence for this classification.

## Next bounded Slice

Build an offline arm which starts from a wall-feasible geometric seed or an
alternate nonlinear tangent and then runs the unchanged exact wall/dynamic and
terminal proofs. Production authority remains frozen until that arm returns a
complete `ManeuverBundle`.

Candidate outcomes:

- alternate tangent succeeds: replace post-hoc wall repair with a bounded
  feasibility-restoration/multi-SQP construction;
- alternate tangent fails but nonlinear solve succeeds: current affine
  single-SQP model is insufficient;
- nonlinear solve also fails: classify the snapshot as physically infeasible;
- solve succeeds but exact proof fails: wall model/certificate mismatch.
