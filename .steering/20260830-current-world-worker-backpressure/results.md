# Results

## Static verification

- `make autoware-build`: 25 packages completed.
- `test_latest_only_worker`: 14/14 passed while the experiment was present.
- `test_single_authority_source_contract.py`: 92/92 passed.
- full `multi_purpose_mpc_ros` package: 2263 tests, 0 errors, 0 failures.

## Dynamic verification

Valid run: `output/20260830-200852`

The queue contract behaved as designed:

- canonical `replaced=0`
- `pending_full_rejected` increased
- the 40 Hz callback remained non-blocking

However, the original failure signature recurred:

1. decision 4017 published emergency Stop during committed ShiftOut;
2. decision 4018 published a certified same-side ShiftOut solution;
3. decision 4019 lost normal authority again;
4. the vehicle entered `ShiftOut -> Recovery` with
   `actual footprint wall margin violated`.

The experiment therefore did not satisfy its dynamic acceptance criterion.
The implementation was removed rather than retained as another partial patch.

## Frozen comparison

Snapshot:

`output/20260830-200852/d1/mpcc_architecture_snapshots/000000004017-ee88c9e56718aeeb-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

Unlike decision 7602, every tested candidate family failed at the common
terminal successor boundary. The persistent, stateless current-world and
production-left primary trajectories solved, but all were rejected because
the track-reference terminal Stop suffix made physical wall contact near
waypoint 318. No comparison arm produced a certified ManeuverBundle.

## Conclusion

Pending replacement is observable load pressure, but it is not the root cause
of this reproduced episode. The common failure owner is the terminal Stop
successor path/certificate shared by otherwise feasible primary trajectories.
The next Slice must audit that owner. It must not reintroduce queue policy,
cadence, timeout, solver or clearance changes.
