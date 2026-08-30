# Results

## Root cause

The serialized Stop command and the plant response had been stored in one
`acceleration_mps2` field.  When the predicted speed reached the zero-speed
floor, the physical adapter changed that field from the serialized braking
command (`-3.0 m/s^2`) to the effective saturated acceleration (`0.0 m/s^2`)
without advancing the command interval.  The exact Stop trajectory remained
physically valid, but immutable command reification correctly rejected the
artifact as `command-changed-within-interval`.

## Change

- Preserve the serialized command in `ActuationSample::acceleration_mps2`.
- Record the saturated plant response independently in
  `ActuationSample::effective_acceleration_mps2`.
- Continue to require one immutable serialized command per command interval.
- Validate that both command and effective acceleration provenance are finite.
- Replace a positional test fixture with named field assignment so provenance
  additions cannot silently shift unrelated values.

No Mission resume rule, lease, grace period, timeout, fallback, solver
tolerance, wall clearance or vehicle clearance was changed.

## Static verification

- `make autoware-build`: passed.
- Focused physical-adapter, Stop-bundle and source-contract tests: passed.
- Full `multi_purpose_mpc_ros` package test suite: 2258 tests, 0 errors,
  0 failures, 0 skipped.
- The pre-existing missing `joycon_contract_guard/package.xml` diagnostic is
  unrelated to this Slice.

## Dynamic verification

Run: `output/20260830-185915` using `make dev2`.

- Certified Stop successor bundles published through canonical normal
  authority: 5.
- `bundle_detail=command-changed-within-interval`: 0.
- Every observed Stop successor production event was `bundle=available`,
  `joined=1`, `join_reason=accepted`, `authority=canonical-normal`.
- Two independent episodes completed
  `ShiftOut -> Pass -> Return -> Idle` during the same run.

The run was stopped cleanly with `make down` after the extended observation
window.

## Independent remaining failures

The run still contains failures outside this Slice:

- committed Pass longitudinal progress stall;
- same-target Mission total-budget expiry;
- actual-footprint wall-margin violation and Recovery.

Those failures occur after successful Stop command reification and therefore
must be investigated as separate root-cause Slices.  They were not hidden by a
parameter adjustment or an additional fallback here.
