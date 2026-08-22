# Validation

## Observed phenomenon

The pre-fix pure selector accepted both invalid cases:

- a retained Track problem under a current Cruise supervisor intent;
- a fresh Track problem under a current Follow supervisor intent.

The first returned `RetainedCertified`; the second returned `FreshCertified`.

## Causal chain

```text
current supervisor intent not represented in authority request
-> candidate qualifier checks only the candidate's historical problem intent
-> current physical decision proof is mistaken for complete current provenance
-> an old Track/Cruise plan can survive an intent transition
-> future final-publisher connection could emit a normal command for the wrong intent
```

## Root cause

`CanonicalNormalAuthorityRequest` did not contain current supervisor intent, so
`resolve_canonical_normal_authority()` could not enforce objective/intent compatibility.

## Evidence

- Baseline: `f9272d0e0e92582b69f5f4016ce38f60c5d6cdbf`.
- Source producer: the baseline request contained current decision/time and candidates only.
- Source detection gap: candidate qualification accepted any candidate whose own intent was Track
  or Cruise.
- Pre-fix failure command: Docker package build followed by the two new filtered tests.
- Pre-fix result: 0/2 passed. The retained case returned `RetainedCertified`; the Follow case
  returned `FreshCertified`.

## Existing patches and masks

- The runtime Track/Cruise path remains shadow-only, so `authority=shadow, selected=0` masks the
  defect from final command output while migration is incomplete.
- Execution decision revalidation proves current physical safety, not current supervisor intent;
  it was retained and no longer carries an implied intent guarantee.
- No fallback, hold, grace, retry, clamp or feature flag was added.

## Implemented correction

1. Added explicit `current_intent` to `CanonicalNormalAuthorityRequest`.
2. Made unsupported current intent an invalid request with Emergency Stop as the existing
   fail-closed destination.
3. Added `IntentMismatch` and required exact equality between candidate problem intent and current
   intent for fresh and retained candidates.
4. Passed `result.intent` from the existing Track/Cruise shadow producer.
5. Appended the new enum value instead of renumbering existing values after an intermediate test
   exposed old-install/new-build ABI mixing.

## Removed or simplified paths

None. This was a missing producer invariant, not an obsolete fallback. New normal authority count
and configuration count are both zero.

## Verification

- Focused `test_mpcc_execution_contract`: 39/39 passed.
- Focused `test_canonical_execution_plan`: 11/11 passed.
- `make autoware-build`: 25 packages passed.
- Package tests: 35/35 CTest targets passed; 1,571 tests, zero errors/failures/skips.
- `git diff --check`: passed.
- Static authority audit: all Track/Cruise canonical telemetry remains
  `authority=shadow, selected=0`; no final publisher connection was added.

Environment-only observations:

- Host `pre-commit` is not installed.
- A repository-wide `clang-format --dry-run` with the container default style reports the existing
  files wholesale, including untouched lines, so it is not a meaningful changed-line gate for this
  repository. Build and test formatting/lint targets passed.

## Remaining concerns

- No dynamic run was started because AWSIM is an external GUI boundary and was not running.
- Retained execution still lacks progress-aligned current wall/dynamic-obstacle revalidation.
- The final publisher still consumes legacy Track/Cruise normal control; authority promotion was
  intentionally not performed.
- The large controller translation unit makes a small header change require an approximately
  five-minute rebuild. This is a maintainability observation, not part of this slice.

## Next dynamic run

Run a clean two-lap `make dev` and confirm:

```text
physically certified == canonical extracted == canonical stored
                     == cursor available == candidate accepted
                     == fresh authority ready == actuation extracted
actuation_diff == 0
authority=shadow
selected=0
```

Also record callback/solve p95, p99 and maximum. A mismatch blocks retained revalidation and any
authority promotion.
