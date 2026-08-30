# Results

## Static verification

- `make autoware-build`: passed, 25 packages.
- Focused retained/source-contract tests: passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 2256 tests, 0 errors, 0 failures,
  0 skipped.
- The unrelated pre-existing `joycon_contract_guard/package.xml` diagnostic
  remains outside this Slice.

## Dynamic evidence

- Command: `make dev2`
- Run: `output/20260830-184601/d1/autoware.log`
- Production authority and all runtime parameters were unchanged from
  `300e987f`.

The run reproduced accepted and rejected Stop reification in the same
ShiftOut encounter.  Accepted examples included decisions 1681, 1743, 1761
and 1767.  Rejected examples were classified at decisions 1741 and 1772 as:

```text
bundle=invalid-actuation-sequence
bundle_detail=command-changed-within-interval
observed=3.000000000
required=0.008831697 or 0.008973632
```

## Root cause

The failure is not progress regression, candidate generation, wall clearance,
solver tolerance or physical Stop infeasibility.  The exact current-world Stop
trajectory is accepted, but its dense rollout changes acceleration from the
maximum braking command (`-3 m/s2`) to zero after velocity reaches zero while
keeping the same `command_interval_index`.  The bundle correctly refuses to
represent those two physical inputs as one immutable serialized command.

The causal chain is:

1. a Stop command interval is requested for a bounded duration;
2. the nonlinear integrator reaches zero speed before that interval ends;
3. the integrator changes negative acceleration to zero for its remaining
   dense samples without ending the command interval;
4. the physical Stop and wall/dynamic proofs remain valid;
5. reification detects two accelerations under one command identity and stays
   fail-closed on external Emergency.

This is a representation/segmentation defect at the Stop rollout producer,
not a reason to relax the bundle certificate.  The next production Slice
should split the serialized interval at the zero-speed event (or terminate the
braking artifact there) so every artifact stage has exactly one command.  It
must not hide the defect by accepting command mutation or averaging the two
accelerations.

## Scope

This Slice adds diagnostic provenance only.  It does not change authority,
Store mutation, commands, solver behavior, Mission lifecycle, timing,
clearance, tolerance, fallback, lease, grace or timeout.
