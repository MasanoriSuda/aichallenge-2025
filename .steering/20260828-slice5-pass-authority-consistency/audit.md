# Audit

## Observation

The production Pass retained a previously certified lateral prefix while a new
target-bound problem was being solved. During that hold, the current target
prediction changed from clear to overlapping. The front cap was correctly
re-applied and pre-contact escape was correctly selected, but a later block
raised the speed reference and floor back to the hold-start speed.

## Causal chain

1. The current-world footprint sweep becomes non-separated.
2. Committed-Pass policy revokes front-cap release.
3. Pre-contact lateral escape becomes active.
4. Target-bound prefix hold remains active for lateral continuity.
5. An unconditional retained-speed block restores speed authority.
6. The command remains full-acceleration for that cycle.
7. The next cycle's emergency supervisor sees insufficient longitudinal
   reserve and publishes Stop.

## Classification

This is a lifecycle/authority defect: a retained artifact is valid as a
lateral prefix, but its speed retention is not revalidated against the current
dynamic certificate. It is not evidence for changing solver tolerance or
physical clearance.

## Implemented repair

- Added a pure speed-retention resolver whose authority requires a current
  clear target certificate.
- Kept the certified lateral prefix independent from retained-speed ownership.
- Added a throttled decision log when current dynamic evidence revokes retained
  speed.
- Preserved artifact construction and validation reasons instead of producing
  a default physical-proof failure.

## Verification

- `make autoware-build`: passed (`25` packages).
- `multi_purpose_mpc_ros`: `49/49` CTest targets passed; the aggregate report
  contained `2007 tests, 0 errors, 0 failures`.
- Dynamic Gate: `output/20260828-024009`.
  - Three ShiftOut episodes were observed.
  - This run did not reach Pass, so the frozen same-cycle pre-contact sequence
    was not repeated dynamically.
  - The new artifact provenance exposed
    `progress-dynamics-mismatch` rather than the previous ambiguous
    `physical-proof-rejected` classification.
  - The run exposed a distinct candidate/continuation defect: ShiftOut
    repeatedly lost a wall-feasible successor and never reached Pass/Return.

The speed-authority rule itself is covered by deterministic tests. Dynamic
Pass/Return remains open and must be handled as the next structural Slice, not
hidden by changing clearance or timeout values.
