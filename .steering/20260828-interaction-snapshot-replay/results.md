# IM-1 results

## Outcome

Accepted.  A rejected Overtake solve can now carry one immutable replay input
containing the exact seven-state semantic request, physical wall proof inputs
and the complete current V2X observation used at the same observation
generation.  The recorder persists that input beside the existing exact QP;
the loader accepts it only when completeness and the recomputed interaction
fingerprint both pass.

## Root-cause finding

The previous artifact was not an architecture-comparison snapshot.  It could
replay the final assembled QP, but A/B/C/D could not rebuild alternative
candidates because all-vehicle state and measured-to-control wall inputs were
not loadable.  Reconstructing those fields from Mission state or a later live
callback would compare different worlds and could misclassify a lifecycle
defect as a solver or feasibility defect.

## Implemented contract

- The async source snapshot owns a sorted all-vehicle `ReplayWorld`.
- The producer binds that world only when its observation is current and keeps
  its original observation generation.
- Wall occupancy, footprint, course-frame knots, current pose and control
  prefix are serialized as proof inputs.
- The interaction seal covers identity, semantic request, path/wall inputs and
  all ordered obstacle observations.
- The loader recomputes the seal; it never trusts a serialized readiness flag.
- Existing exact-QP v1 artifacts remain readable by `mpcc_qp_replay`, while an
  incomplete artifact is explicitly rejected for interaction replay.

## Authority and complexity audit

- Production command authority changed: no.
- Publisher, Gate A, certified-plan store or retained-plan API added: no.
- Solver invocation, tolerance, clearance, timeout, lease or fallback added:
  no.
- New exceptional production path: zero.  Missing or mixed-epoch world data
  merely leaves an observation artifact incomplete; it cannot affect command
  selection.
- Old authority removed: none; this Slice is observation-only by design.

## Verification

- Failure-first compile initially failed on the absent replay contract.
- `make autoware-build`: passed, 25 packages.
- Focused `test_mpcc_architecture_snapshot`: passed.
- Full `multi_purpose_mpc_ros` package test: 49/49 CTest targets passed,
  1981 tests, zero failures.
- Mutation checks prove that changing vehicle state, wall bounds, identity or
  semantic initial state invalidates the interaction seal.
- `git diff --check`: passed.

## Dynamic evidence boundary

The unit test generates and reloads one complete failure artifact.  A native
live failure and `.steering/ano` encounter import are intentionally deferred to
the later capture/parity Slice; no production solve was changed merely to
force a failure in IM-1.
