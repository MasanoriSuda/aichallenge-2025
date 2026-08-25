# Evidence

## Entering dynamic evidence

`output/20260825-215909` submitted 23 execution drafts and replaced 12 pending
jobs, but exposed only two completed results. Both appeared only after the
live tactical selection was unavailable. The result mailbox required exact
equality with `latest_submitted_sequence`, unlike the canonical helper whose
test explicitly permits completion sequence 10 while sequence 11 is queued.

## Static validation

- `make autoware-build`: 25 packages passed.
- Full `multi_purpose_mpc_ros` test: 49/49 targets, 1,875 tests, zero
  errors/failures/skips.
- Source contract proves the execution mailbox uses
  `should_publish_latest_only_result()`, carries context epoch, contains no
  equality-to-latest publication rule and remains disconnected from
  production publication.
- The direct host pytest collection still fails before the selected test on
  the known `localization_scope` PYTHONPATH dependency. The canonical Docker
  test includes that module and passed all targets.

## Corrected bounded run

- Command: `make dev2`.
- Artifact: `output/20260825-221447`.
- Domain 1 produced results while submissions were still active:
  - first result: submitted=2, result=1, exact tactical identity,
    current-world accepted, authority-ready=1;
  - later aggregate: submitted=34, replaced=12, result=21, complete=20,
    current-world join=6, authority-ready=4;
  - logged snapshot cost 0.110--0.136 ms, worker cost 39.454--40.960 ms,
    result age 0.050--0.065 s;
  - all logged epochs matched (`593/593`) and all logged selections were exact
    same-side current tactical authority.
- Current-world `dynamic-path-blocked` correctly rejected later physically
  obsolete trajectories without erasing the newer completion stream.
- Every callback telemetry window reported zero overrun. During execution
  worker activity the observed callback maxima were 3.435--4.869 ms; the
  bounded run maximum was 16.438 ms during earlier recovery monitoring.
- The end-of-run odometry stale, RViz/relay exit and orchestrator stop errors
  followed the deliberate `make down` boundary and are not execution-worker
  failures.

## Verdict

PASS. The transport defect is removed: a current context can now expose and
join completed six-state execution candidates before its tactical selection
disappears. This remains shadow evidence; no Mission, normal store or command
authority changed. The next promotion Slice still must obtain the required
intent coverage and delete the five-state Gate A in the same authority change.
