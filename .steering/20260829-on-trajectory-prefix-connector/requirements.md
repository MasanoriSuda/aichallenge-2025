# Requirements

## Objective

Replace the remaining asynchronous candidate splice defect with an explicit
on-trajectory connector.  A newly solved normal MPCC trajectory must start
from a state which the last actually published certified artifact is scheduled
to reach; elapsed wall time may not stand in for execution of an unpublished
prefix.

## Frozen evidence

- `output/20260829-022011` classified the failure as a scheduling/connector
  defect: a candidate was physically joinable at its solve origin, but became
  steering- and lateral-state-incompatible while the worker ran.
- Time-aligning an unpublished candidate to an elapsed suffix is safe only
  when the existing current-world physical proof accepts it.  It does not make
  the skipped controls causal and therefore is not the complete architecture.
- The upper-ranked log keeps asynchronous work tactical and continuously
  solves/executes its main GMPCC; AS-RTI and ASAP-MPC explicitly reconnect an
  asynchronous solve to the latest state or currently executed trajectory.

## Invariants

- Production authority, solver tolerances, wall clearance, Mission timing and
  fallback policy remain frozen during the connector comparison.
- The parent is the last actually published certified artifact, never merely
  the latest solved candidate.
- Parent identity, first-publication clock/cursor and planned switch point are
  immutable inputs to the connector proof.
- Candidate state at the switch and parent state at the same control time must
  be compared in one physical/control coordinate contract.
- A shadow connector cannot construct a command or production authority.

## Exit criteria

- Every asynchronous result is classified as on-parent-trajectory,
  off-parent-trajectory, parent-unavailable or switch-unavailable.
- The classification is reproducible from immutable artifacts without reading
  mutable Mission geometry.
- Dynamic evidence identifies whether a scheduled committed prefix is enough
  or a latest-state feedback QP is required.
- Only after that evidence may the connector be added to production and the
  elapsed-suffix-only adoption path be removed in the same Slice.
