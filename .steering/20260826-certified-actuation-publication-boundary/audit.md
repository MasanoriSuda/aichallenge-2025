# Root-cause audit

## Observed phenomenon

`output/20260826-103853/d1/autoware.log` enters the race session at line 499.
The first post-start normal failure is decision 898 at line 505.  Its atomic
transition admission at line 504 reports `solver=solved`, `physical=accepted`
and `world=accepted`, but `joined=0`.  The final decision reports
`production_reason=command-rejected` and publishes canonical Emergency.

## Authority graph

```text
Track -> Cruise semantic request
  -> synchronous six-state solve: solved
  -> immutable physical proof: accepted
  -> exact current-world revalidation: accepted
  -> command candidate: finite and identity-complete
  -> canonical command lower-bound check: rejected
  -> production authority: unavailable
  -> explicit Emergency: published
```

## Root / contributor / mask / recovery

- Root: certified solver residuals are interpreted with exact-zero semantics at
  the production command boundary.
- Contributor: race start and stop/hold horizons place predicted speed or
  virtual progress on a zero lower bound.
- Mask: explicit Emergency correctly prevents an uncertified command, but makes
  the symptom look like launch, V2X, or Recovery failure.
- Detection gap: previous tests closed artifact and physical-trajectory
  boundaries but did not cross the final production adapter.
- Recovery: not involved in the first incident.

## Competing hypotheses

1. Wall or dynamic obstacle rejection: falsified by `world=accepted` and no
   blocker.
2. Solver failure: falsified by `solver=solved` and `physical=accepted`.
3. Incomplete identity: low probability because the exact certified plan and
   current-world proof share sequence 304; the command builder's remaining
   rejectable fields are the raw lower-bound actuation values.
4. Certified lower-bound residual mismatch: supported by the prior two
   boundary failures and by the exact command-builder predicates.

## Implementation gate

- new branches/configuration: zero
- new fallback/authority: zero
- parameter changes: zero
- replaced assumption: certified stop-state residuals require exact floating
  point zero at publication
- remaining legacy authority: unchanged in this Slice
- rollback commit: `005505e`

## Implemented correction

The production adapter now performs one typed conversion from the certified
numerical representation to the exact physical actuator representation.  A
finite predicted speed inside the exact physical trajectory's sealed lower-
bound tolerance, and a finite virtual-progress speed inside the execution
artifact's sealed global tolerance, are projected to zero.  Values below those
certificates remain fail-closed.  The immutable execution artifact and physical
proof are not modified, and the same projected speed is used by the canonical
command and compatibility horizon.

This replaces the contradictory exact-zero assumption at publication.  It
does not add a second owner, retry, fallback or special race-start path.

## Verification

- The new test failed before the correction with `command-rejected` for both
  the certified and uncertified residual cases.
- After the correction, the certified residual publishes exact zero and the
  residual below the physical certificate is rejected as `invalid-actuation`.
- `make autoware-build` passed for all 25 packages.
- The complete package suite passed 51/51 tests, including 56 source-authority
  contract checks.
- Moving acceptance `output/20260826-111752` shows both domains entering the
  race with no `command-rejected` occurrence.  Domain 2 joins the first
  Track-to-Cruise six-state plan directly.  Domain 1 reaches a different typed
  boundary (`world=steering-unreachable`) because the future physical steering
  origin is interpreted as the immediate publication command.  That separate
  time-base defect is not masked by this correction.
