# Results: dynamic-wait prefix owner removal

## Implementation

- `DynamicMissionWaitAction::Hold` remains a tactical no-transition result.
- `publish_dynamic_wait_forward_prefix()` remains available as an optional
  reference producer, but its failure can no longer mutate `FollowPrepare` to
  `Recovery`.
- Hard-fault `DynamicMissionWaitAction::Recovery` and the Emergency supervisor
  were not changed.
- No solver setting, clearance, weight, timeout, lease, retry, grace, fallback
  or configuration was changed.

## Static acceptance

- `make autoware-build`: 25 packages passed.
- `ctest --test-dir .../multi_purpose_mpc_ros --output-on-failure`: 54/54
  passed.
- `test_single_authority_source_contract.py`: 76/76 passed, including the new
  Hold ownership contract.

## Dynamic acceptance

Run: `output/20260830-001650`, Domain 1, bounded `make dev2`.

| Observation | Baseline `20260829-235457` | Candidate `20260830-001650` |
|---|---:|---:|
| `ShiftOut -> FollowPrepare` | not used as denominator | 3 |
| `FollowPrepare -> Recovery` | 5 | 0 |
| obsolete prefix-failure Recovery reason | 4 | 0 |
| dynamic-wait release to fresh search | 6 | 3/3 waits |

In all three candidate waits, the current-world canonical ShiftOut artifact
continued to own the published normal command while the optional reference was
unavailable. The existing wait budget then released the encounter to fresh
search. The removed legacy prefix edge did not reappear.

## Scope and residual failures

This Slice is accepted only for the split-owner defect. The run still recorded:

- one independent `Pass -> Recovery` caused by
  `actual footprint wall margin violated`;
- 52 Emergency-override decisions out of 173 logged control decisions,
  including pre-session and moving-authority gaps;
- 21 aggregate callback reports with one or more overruns;
- one `ShiftOut -> Pass`, but no `Pass -> Return -> Idle` completion.

Those failures do not falsify the ownership correction, but they prevent an
Overtake or integration-quality pass. They remain separate root-cause Slices;
they must not be hidden by restoring the deleted prefix authority or by tuning
solver/clearance parameters.
