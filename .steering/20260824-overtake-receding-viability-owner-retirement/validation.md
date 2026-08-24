# Validation

## Static and package gates

- `test_single_authority_source_contract.py`: 15 passed.
- `make autoware-build`: 25 packages passed.
- `multi_purpose_mpc_ros` package tests: 40/40 CTest targets passed;
  1738 tests, 0 errors, 0 failures and 0 skipped.
- The pure authority resolver covers canonical/reference-complete demotion,
  non-canonical intent, incomplete references and every independent hard-fault
  input.
- The source contract proves the demoted branch cannot transition phase, enter
  DynamicWait, arm retry-block or reset the Mission.

## Dynamic run

Run: `output/20260824-132703`

The clean run reached a generation-1 ShiftOut and published certified
canonical retained authority. It did not naturally exercise the demoted
legacy physical-revalidation branch because an earlier legacy owner fired:

```text
runtime wall escape prefix unavailable
-> Mission generation invalidated
-> ShiftOut -> FollowPrepare / DynamicMissionWait
```

The run therefore provides two bounded results:

1. no `optimized horizon failed physical revalidation -> DynamicWait/Recovery`
   transition occurred;
2. after the earlier invalidation, canonical physical proof detected actual
   hard wall contact and selected explicit Emergency. The new resolver did not
   suppress the independent hard-fault boundary.

The exact demotion path is accepted by deterministic resolver and source-
deletion gates, not claimed as naturally exercised dynamic evidence.

## Earliest remaining break

At `1787545717.382770`, the runtime wall preplan could not produce an escape
prefix because its legacy lateral-acceleration approximation rejected the
reference. It then invalidated generation 1 even though the decision trace
reported `hard_fault=0` and cross-side evaluation was still in flight. That is
a separate, earlier Mission-viability owner. It must be audited in a new Slice;
it is not folded into this change.

## Acceptance

Accepted as a structural owner deletion with partial dynamic coverage:

- canonical normal authority and explicit Emergency remain the only command
  owners in the changed branch;
- legacy receding validation remains reference/corridor telemetry;
- missing reference geometry and independent hard faults stay fail-closed;
- no parameter, grace, lease, retry or fallback was added.
