# Validation

## Static validation

- `git diff --check`: passed.
- Host source-contract test with third-party plugin autoload disabled:
  28 passed.
- Targeted retained dynamic-world test: 14 passed.
- Docker package build: passed.
- Docker package tests: 48/48 targets passed.
- `colcon test-result --all`: 1879 tests, 0 errors, 0 failures and
  0 skipped.  It also reports the pre-existing stale
  `joycon_contract_guard/package.xml` build-tree traceback.

## Dynamic validation

Run: `make dev2`

Artifact: `output/20260825-064109`

- Both domains received one same-generation peer snapshot and executed the
  new retained dynamic-world shadow proof.
- d1 accepted clear suffixes, then rejected a closing suffix as
  `dynamic-path-blocked` with `blocked_by=d2` and minimum signed clearance
  `-0.006 m`.  The physical reason was preserved instead of being overwritten
  as invalid input.
- d2 continued to accept retained suffixes while the peer was physically
  clear; representative minimum clearances increased from about `0.4 m` to
  more than `26 m` as separation grew.
- The evaluator was normally about `0.04--0.23 ms`; the largest telemetry
  window sample observed was `1.337 ms`, well below the 25 ms control period.
- d2 recorded no nonzero callback-overrun window.  d1 recorded two overrun
  windows with MPCC work at up to `65.957 ms`; retained evaluation in the
  adjacent window was at most `0.269 ms`.  This is an existing MPCC runtime
  quality issue, not evidence that the new dynamic proof caused the overrun.
- The proof remained `authority=shadow, selected=0`; no production command
  source or parameter changed.

## Conclusion

The empty-world proxy has been replaced by the required physical predicate:
an observed peer may coexist with retained execution only when every sampled
point of the joined suffix remains non-overlapping.  Missing, stale, mixed or
invalid observation remains fail-closed.  Dynamic Acceptance and dynamic
blocking are both demonstrated, but production promotion remains a separate
Slice because Track/Cruise retained authority and legacy-owner deletion must
be performed atomically.
