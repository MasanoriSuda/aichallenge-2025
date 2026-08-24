# Validation

## Static identity

- Frozen source baseline: `174b2682312765743f92563f1bf25f039a80ab3f`.
- `make autoware-build`: PASS, 25 packages completed.
- Source deletion contract:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_single_authority_source_contract.py`:
  PASS, 9 tests.
- Package test result:
  `colcon test-result --verbose --test-result-base build/multi_purpose_mpc_ros`:
  PASS, 1712 tests, 0 errors, 0 failures, 0 skipped.
- The dynamic Gate will use the install tree rebuilt by this Slice. The host-visible
  install symlink is container-absolute, so host-side string inspection is not an
  identity proof; the successful rebuild and subsequent runtime source marker are
  the accepted provenance join.

## Dynamic Gate

- Run: `output/20260824-085556`, Domain 1, bounded `make dev2`.
- ShiftOut: EXERCISED; sampled final decisions were 3 certified canonical, 15 explicit Emergency,
  0 legacy normal.
- committed DynamicWait: EXERCISED once; preserved ShiftOut intent and failed explicitly through
  canonical Emergency.
- Pass: `NOT EXERCISED`.
- Return: `NOT EXERCISED`.
- validated DynamicEscape: `NOT EXERCISED`.
- line Recovery/Rejoin: EXERCISED and REJECTED; 3 sampled final decisions were
  `legacy-normal-bypass` / `legacy-spatial-mpc-3state`.
- The callback showed a sustained overrun interval while old Recovery normal control was active,
  peaking at 30.213 ms in the one-second aggregates. This is contributing runtime evidence, not the
  cause of the deterministic Rejoin domain omission.
- No `overtake-wall-admission-hold` legacy owner was observed.

## Acceptance decision

REJECTED. The first violated invariant is the missing canonical `Rejoin` production domain. Pass,
Return and DynamicEscape remain `NOT EXERCISED`; their absence is not accepted as evidence.
