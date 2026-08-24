# Validation

## Static

- `make autoware-build`: PASS, 25 packages.
- `test_mpcc_execution_contract`: PASS.
- `test_race_mpcc_foundation`: PASS.
- `test_single_authority_source_contract.py`: PASS, 10 tests.
- Full `multi_purpose_mpc_ros` package test: PASS, 40/40 CTest targets.
- `git diff --check`: PASS.

The first build attempt found an incorrectly placed Rejoin-only warm-publication
guard in the Follow path. It was corrected before the accepted build. The final
Follow path is unchanged; only Rejoin shadow suppresses publication to the
global certified warm-start store.

## Dynamic

- Run: `output/20260824-092036`, Domain 1, bounded `make dev2`.
- ShiftOut: EXERCISED.
- Rejoin shadow: `NOT EXERCISED`.
- Rejoin production authority: unchanged legacy boundary.
- Production behavior change from this Slice: none.

The observed Overtake episode entered ShiftOut at waypoint 112 and was reset to
Idle by external Stuck/AWSIM Recovery at waypoint 117. It did not enter the line
Recovery phase that maps to Rejoin.

## Acceptance

The observation infrastructure is ACCEPTED. Rejoin production promotion is
REJECTED because the required dynamic phase was not exercised. The next Slice
must address the earlier canonical ShiftOut async-availability failure, not tune
Rejoin or add a fallback.
