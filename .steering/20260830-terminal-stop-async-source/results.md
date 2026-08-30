# Results

## Ownership audit

The normal seven-state population already runs in `LatestOnlyWorker` and
places accepted normal plans in the existing certified Store.  By contrast,
the published Stop successor is built synchronously in
`rate_resolved_normal_production_control()` only after normal authority is
lost, then immediately rejoined to canonical authority.

The control callback is therefore the wrong integration point for the Stop
lattice.  Adding a lattice solve there would create another fallback and make
the callback tail worse.  A future live comparison must use a separate
latest-only observation worker and bind its result to the exact normal source
epoch.

## Implemented boundary

`mpcc_rate_resolved_stop_control_lattice` now owns the deterministic parts of
the candidate source:

- exact publisher-interval rebasing;
- solver-safe maximum-braking velocity equalities;
- replay-world progress/time rebasing;
- deterministic horizon-relative steering-rate lattice generation.

The component cannot solve, prove, write the Store or publish.  The
architecture comparison now consumes this component instead of keeping a
private copy of the same policy.

## Frozen replay

The refactor preserved the earlier classification:

| Snapshot | Lattice result | Candidate | Exact reserve |
|---|---|---|---:|
| decision 4017, ShiftOut positive | accepted | positive 3 / negative 3 / hold, #8 | 0.374927 m |
| decision 4489, Pass negative | accepted | positive 3 / negative 3 / hold, #8 | 0.0693029 m |

Both retain terminal velocity approximately zero and pass the unchanged exact
wall/current-world proofs.  Production authority was not changed.

## Verification

- `make autoware-build`: passed (25 packages)
- focused architecture-comparison CTest: passed
- frozen decision 4017 replay: accepted with the same lattice schedule
- frozen decision 4489 replay: accepted with the same lattice schedule
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed
- `git diff --check`: passed

## Remaining boundary

This Slice does not yet provide live dynamic evidence.  The next Slice may add
one observation-only latest-only worker fed after normal certification.  It
must not delay Store admission, and it must not be consumed by the publisher.

`Return` also remains intentionally unsupported by the current Stop bundle:
rest alone does not prove a valid Return successor.  That semantic successor
must be designed before production promotion.
