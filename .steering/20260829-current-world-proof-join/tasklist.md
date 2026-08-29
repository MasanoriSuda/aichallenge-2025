# Task list: current-world proof join

- [x] Freeze parameters and production authority.
- [x] Share exact Frenet-to-world pose reconstruction.
- [x] Share immutable replay-world dynamic proof.
- [x] Replace architecture-comparison duplicate proof.
- [x] Add proof unit tests and replay frozen snapshots.
- [x] Require joined proof before Overtake Store replacement.
- [x] Integrate proof-guided SQP without a parallel production branch.
- [x] Run focused tests and full build.
- [x] Run `make dev2` and inspect decision/proof telemetry.
- [x] Document dynamic evidence and upper-entry comparison.
- [x] Commit the reviewed slice.

## Verification record

- C++ focused tests: physical wall 11, dynamic proof 3, dynamic obstacle 19,
  shadow 39, stateless maneuver 17, architecture comparison 14 -- all passed.
- Source-contract tests: 73 passed.
- Full `make autoware-build`: 25 packages passed.
- Frozen replay: snapshot 1612 left becomes certified at depth 1; its right
  side remains depth 0.  Snapshot 1675 keeps its depth-0 left certificate and
  its blocked right side remains rejected.
- Dynamic `make dev2`: exact joined proof generated one valid depth-0 candidate;
  Gate A correctly rejected its stale steering join.  No depth > 0 live event
  occurred, and production Overtake authority was not entered during this run.
- The run also reproduced pre-existing Track/Cruise authority loss and Recovery
  churn seen in earlier 20260829 runs.  This slice does not relabel that separate
  integration defect as an Overtake dynamic-proof regression.
