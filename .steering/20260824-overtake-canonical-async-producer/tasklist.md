# Tasklist

- [x] Add and run failure-first async context lifecycle tests.
- [x] Implement exact-intent context transition/invalidation contract.
- [x] Use the contract for the accepted Follow producer without behavior change.
- [x] Add dedicated Overtake shadow solver/plan lifecycle and worker.
- [x] Consume results only through current identity and current-world proof.
- [x] Remove synchronous canonical telemetry construction from production path.
- [x] Add compact producer/coverage telemetry.
- [x] Run focused tests, package tests and build.
- [x] Make the worker fresh-chain use only the sealed snapshot context; dynamic
      gate found clone-local intent re-derivation rejecting every ShiftOut job.
- [x] Run a dynamic Overtake shadow gate and record the result.
- [x] Commit without result-summary.json.
