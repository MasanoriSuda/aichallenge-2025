# Validation

## Static evidence

- Failure-first `make autoware-build`: failed at link time on every new retained-window/proof API,
  demonstrating that the former plan-ID/stage-index/boolean contract could not express Gate B.
- Implementation `make autoware-build`: 25 packages passed.
- Focused canonical tests: 3/3 CTest targets passed.
- Full `multi_purpose_mpc_ros` package: 36/36 CTest targets passed.
- Aggregated result after the first full run: 1610 tests, zero errors/failures/skips.  The existing
  stale `joycon_contract_guard/package.xml` result warning is unrelated to this Slice.
- `ament_uncrustify`: no divergence in the three new C++ files.

The pure suite covers seven grouped tests, including exact partial-stage timing, explicit circular
progress lift, stage-index/progress alias rejection, current provenance invalidation, separate
delay/connector rejection, moving-obstacle rejection, missing-input rejection and fingerprint
mutation.

## Deletion/replacement audit

- `CanonicalExecutionRevalidation` remains the fresh same-callback summary used by the fresh
  Track/Cruise shadow path.
- Its builder now rejects a certificate decision which differs from the solved problem decision;
  it can no longer manufacture a retained candidate.
- Retained candidates can be built only by `build_canonical_retained_candidate()` after validating
  the sealed current-observation proof.
- No fallback, flag, timeout, lease, retry, parameter or final authority was added.

## Dynamic evidence

Pending for the runtime-adapter portion.  Published control remains the legacy production path
until a later explicit authority-promotion approval.

## Rejected alternatives

- Reusing the old plan's physical certificate.
- Pairing retained stage `k` with current horizon stage `k`.
- Weakening canonical semantic residual admission.
- Adding a timeout, retry, lease or last-command fallback.
