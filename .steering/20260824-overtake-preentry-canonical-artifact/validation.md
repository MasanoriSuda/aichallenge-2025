# Validation

## Static Gate

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q .../test_single_authority_source_contract.py`:
  12 passed.
- `git diff --check`: pass.
- `make autoware-build`: pass, 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 tests passed.
- `colcon test-result --verbose`: 1,755 tests, 0 errors, 0 failures, 0 skipped.
- Unrelated environment warning: stale `build/joycon_contract_guard/package.xml` could not be
  collected.

## Dynamic Gate history

### Rejected Gate 1: missing current target contract

- Run: `output/20260824-095518`
- Entry reached ShiftOut, but first current-world proof rejected
  `target-release-uncertified`.
- Correction: carry current pre-entry target observation/prediction explicitly rather than reading
  the committed locked-target lifecycle before it exists.

### Rejected Gate 2: missing pre-entry prediction

- Run: `output/20260824-101828`
- All candidate artifacts failed closed with `current target prediction unavailable`; no entry.
- Correction: populate the explicit target snapshot from the selected current V2X vehicle.

### Rejected Gate 3: corridor identity mismatch

- Run: `output/20260824-105259`
- Entry occurred, but immediate retained proof rejected stage 2 with
  `stage-corridor-violation`.
- Root cause: retained proof rebuilt bounds from the later post-transition Mission problem instead
  of the selected plan's solved bounds.
- Correction: make lateral lower/upper state bounds part of `CanonicalExecutionPlan` and slice that
  sealed corridor at the retained cursor.

### Accepted Gate 4: atomic initial Overtake entry

- Run: `output/20260824-110945`
- Domain: 1 (`make dev2`).
- Entry evidence:
  - line 611: selected PassPlan frozen;
  - line 612: `Idle -> ShiftOut`;
  - line 613: exact five-state entry certificate accepted, age 0.120 s, 20 exact stages;
  - line 621: first observed ShiftOut decision published
    `canonical-shiftout-retained`, plan 2309, no entry-start `async-pending` Emergency.
- Acceptance result: the selected dual-branch solution now crosses Mission adoption as one
  immutable executable artifact and owns the first ShiftOut command after current-world proof.

## Separated next defect

At line 636, roughly 0.75 s after accepted entry, retained proof rejected a current progress
discontinuity after the receding-DP source became stale. The runtime failover then replaced the
Mission from side -1 to side +1 at lines 653--654 without carrying the corresponding pre-solved
canonical artifact, causing one `async-pending` Emergency at line 662. Later incoming/stored plans
alternated `initial-corridor-violation`, `invalid-progress-evolution` and retained acceptance.

This is not a regression of the initial-entry artifact contract. It is a distinct execution-time
Mission replacement/progress-origin ownership defect and must be addressed in a new Slice without
weakening the accepted wall/target/corridor proof.

## Configuration and artifacts

- No solver, weight, clearance, cadence, timeout or vehicle parameter changed.
- No new grace, retry, lease, controller fallback or alternate publisher was added.
- `aichallenge/result-summary.json` is user-owned and excluded from this Slice.
