# IM-3 results

## Outcome

Accepted as an offline architecture-comparison component.  One sealed
Interaction Snapshot now evaluates the persistent A candidate and independently
rebuilt B-left/B-right candidates through separate instances of the unchanged
seven-state SQP and the same exact physical proof contracts.

No comparison candidate is connected to production authority.  A
`ManeuverBundle` is data only and exists only after solver, nonlinear exact
trajectory, swept wall, timed dynamic-obstacle and terminal-successor checks
all accept.

## Root-cause finding

IM-1 preserved the control-prefix poses and the footprint already expanded for
QP wall refinement, but not the prefix time axis or the raw physical body.
Consequently an offline candidate could not reproduce production's dynamic
opponent proof, and a wall replay would have applied hard clearance twice.

The earliest violated invariant was replay evidence completeness, not Mission
lifecycle, solver tolerance or clearance magnitude.  The source snapshot
schema is therefore advanced to v2 and seals both missing inputs.

## Implemented contract

- `ReplayWorld` owns exact measured-to-control elapsed time and the raw ego
  footprint.
- Completeness proves the raw/expanded footprint relationship and exact timing
  from observation to control origin.
- Production retained revalidation and offline comparison consume one shared,
  pure dynamic-footprint proof implementation.
- A, B-left and B-right each own a fresh `SolverContext`; no warm start crosses
  an architecture arm.
- Exact Frenet trajectories are reconstructed against the recorded course
  frame and checked against the recorded wall grid and every same-generation
  obstacle.
- Persistent and stateless arms use one terminal-successor resolver.
- `mpcc_architecture_compare <snapshot.yaml>` reports the typed rejection stage
  and comparable solve/terminal/reserve metrics for all arms.

## Failure-first evidence

- Replay round-trip tests first failed because prefix timing and raw footprint
  did not exist.
- Bundle tests prove that a new overlap on the measured-to-control prefix
  rejects every arm after solving and produces no bundle.
- Missing Return/Stop successor and source fingerprint mutation likewise
  produce no bundle.

## Authority and complexity audit

- Production command authority changed: no.
- Controller link to architecture comparison: no.
- Solver setting, weight, horizon, clearance, timeout, lease, retry or fallback
  changed: no.
- Production behavior change: none; the controller only records additional
  observation provenance when a rejected Overtake snapshot is captured.
- Duplicated dynamic proof removed: the retained production path calls the
  extracted common implementation with unchanged semantics.

## Verification

- Focused architecture/snapshot/stateless/retained tests: passed.
- Full package suite: 51/51 CTest targets and 1,992 tests passed.
- `make autoware-build`: passed, 25 packages.
- Central experiment registry validation: passed, 3 snapshots and 13
  experiments.  Pre-existing abbreviated commit IDs and an unregistered unit
  snapshot reference were normalized so the registry is enforceable again.
- `git diff --check`: passed.
- Production target search finds no link or include of
  `mpcc_architecture_comparison`; only its offline CLI and tests link it.

## Dynamic evidence boundary

The deterministic test proves pipeline parity and fail-closed bundle
construction, not whether A or B wins on the racing failures.  Classification
still requires a native replay-ready failure snapshot.  Until then, no Mission
retirement or production promotion is justified.
