# Task list

- [x] Correlate current HEAD with the 2026-08-19 run.
- [x] Make DP optimizer source age an absolute execution limit.
- [x] Add a pure execution-reference trust-envelope helper and tests.
- [x] Apply the trust envelope before the DP reference reaches MPC/MPCC.
- [x] Revoke aggressive DP authority during extended solver degradation.
- [x] Add measured lateral/heading tracking checks to runtime lease renewal.
- [x] Update the MPC integration specification.
- [x] Run focused unit tests: 28/28 passed.
- [x] Run `make autoware-build`: 25 packages passed.
- [x] Review scoped files and exclude the pre-existing `aichallenge/result-summary.json` change.
- [x] Commit scoped files.

## Definition of Done

- A stale or diverged DP path cannot retain execution authority.
- Extended solver fallback uses a bounded Mission reference.
- Focused tests and package build pass.
- Runtime validation remains observable in debug logs.
