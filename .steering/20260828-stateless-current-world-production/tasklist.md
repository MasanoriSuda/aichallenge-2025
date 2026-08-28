# Task list

- [x] Freeze root-cause evidence and the authority boundary.
- [x] Add a pure bounded current-world candidate producer.
- [x] Route synchronous and asynchronous pre-entry SQP through the producer.
- [x] Remove direct persistent-geometry evaluation at that boundary.
- [x] Add source-contract and focused unit tests.
- [x] Replay frozen decision 3931.
- [x] Run build and package tests.
- [x] Run dynamic acceptance and update this file.
- [x] Commit the completed Slice.

## Static evidence

- Docker package build: passed.
- Focused tests: `test_mpcc_stateless_maneuver`,
  `test_mpcc_architecture_comparison` and
  `test_single_authority_source_contract` passed.
- Frozen decision 3931 production-right G: accepted through candidate two,
  `earliest-physical-diagonal`; production-left G remained rejected.

## Dynamic evidence

Run: `output/20260828-162503`

- `candidate=direct-side/1` reached Gate A and was selected.
- D1 completed `Idle -> ShiftOut -> Pass -> Return`; physical separation and
  rear-clear were therefore achieved through the new production boundary.
- The later Return lost canonical authority and entered Emergency/Recovery.
  This is downstream of the replaced pre-entry boundary and is frozen as a
  separate Return root-cause snapshot:
  `000000000773-return-wall-refinement-solve-rejected`.
- A second ShiftOut also produced a separate frozen snapshot at decision
  3023; it is not patched in this Slice.

The Slice is dynamically accepted for its scoped authority boundary.  It is
not evidence that the complete Overtake lifecycle is production-ready.
