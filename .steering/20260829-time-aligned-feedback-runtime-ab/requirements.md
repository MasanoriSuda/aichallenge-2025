# Requirements

## Objective

Close the remaining observation gap for the canonical multirate seven-state
MPCC: determine whether a solved preparation can be joined to the latest
serialized predecessor by rebuilding one common absolute-time suffix, without
changing production authority.

## Frozen evidence

- Baseline commit: `f2c06db4`.
- Run: `output/20260829-175025`.
- D1 repeatedly rejected newly solved candidates as `steering-unreachable`
  after Stop had published several different steering commands while the
  worker was running.
- D2 published canonical seven-state authority under the same formulation.
- Same-cycle full solves have already proved feasibility but exceeded the
  25 ms callback budget.
- Replacing only x0 in the old final QP is already rejected as a mixed-origin
  problem. Projecting only the first steering command remains observation-only
  and is not eligible for promotion.

## Hypothesis

State, input, nominal path, wall rows, dynamic-obstacle rows and phase timing
must advance under one absolute-time suffix. Rebuilding and solving that
prepared suffix from the latest physical state and last serialized input will
produce one internally consistent artifact more cheaply than a second full
current-world solve.

## Constraints

- Production Store, authority selection and publisher remain unchanged during
  the A/B.
- No Mission resume rule, lease, grace period, timeout or fallback is added.
- No solver tolerance, steering rate, clearance or behavior parameter changes.
- The old mixed-origin feedback method remains available only as the A arm;
  it cannot become production authority.
- Dynamic acceptance requires exact nonlinear and physical/current-world
  proof. A solver result alone is insufficient.

## Exit classification

- suffix solve and proof succeed within bounded worker time: design a single
  production feedback connector and delete stale direct adoption atomically;
- suffix solve succeeds but proof fails: model/certificate mismatch;
- suffix problem rejects before solve: clock/provenance reconstruction defect;
- suffix solve remains too slow: use a current-state main MPCC cadence rather
  than adding another lifecycle exception;
- old and suffix arms fail on the same immutable world: candidate/formulation
  or physical infeasibility remains upstream.
