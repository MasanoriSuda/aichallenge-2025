# Requirements

## Purpose

Close the current-intent provenance gap in the runtime-disconnected canonical
Track/Cruise authority selector before any normal-command authority promotion.

## Repaired invariant

A canonical normal candidate may execute only when its sealed problem intent is
identical to the current supervisor intent.  Both values must be Track or Cruise.

## Evidence boundary

- Branch: `develop_july`
- Baseline: `f9272d0e0e92582b69f5f4016ce38f60c5d6cdbf`
- Evidence: source audit and deterministic unit tests only
- Preserved user change: `aichallenge/result-summary.json`
- Dynamic run: unavailable because AWSIM/Autoware is not currently running

## Expected behavior

- Track request + Track candidate: candidate may proceed to the remaining gates.
- Cruise request + Cruise candidate: candidate may proceed to the remaining gates.
- Track request + Cruise candidate, or Cruise request + Track candidate: reject.
- Follow/Hold/Stop/overtake intent request: fail the Track/Cruise selector closed.

## Non-scope

- No final publisher or command-authority change.
- No retained-plan wall/obstacle revalidation implementation.
- No parameter, solver, horizon, clearance or timing change.
- No new fallback, feature flag, grace period or controller branch.

## Acceptance

- A pre-fix deterministic test proves that a currently re-certified retained Track
  plan is incorrectly accepted under a current Cruise intent.
- The same case is rejected after the fix with an explicit intent mismatch reason.
- Invalid non-Track/Cruise current intent fails as `InvalidRequest`.
- Existing focused tests, package tests and `make autoware-build` pass.
- Production command selection remains unchanged (`authority=shadow, selected=0`).
