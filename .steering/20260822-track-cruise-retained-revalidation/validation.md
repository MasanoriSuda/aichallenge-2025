# Validation

## Static evidence

- The first failure-first `make autoware-build` failed at link time on every new retained-window
  and proof API.  The former plan-ID/stage-index/boolean contract could not express Gate B.
- A second failure-first link run with a stub runtime adapter failed on every current-world API.
- Final `make autoware-build`: 25 packages passed.
- Final focused canonical CTest: 3/3 targets passed.
- Final full `multi_purpose_mpc_ros` run: 37/37 CTest targets passed; 1552 tests, zero
  errors/failures/skips.
- `ament_uncrustify`: no divergence in the new header, source and test.
- An accidental whole-file `ament_uncrustify --reformat` of the legacy controller would have
  changed roughly 13,000 unrelated lines.  That formatting churn was removed before the intended
  functional patch was reapplied; the final controller diff is 361 additions and one deletion.

The pure suite covers exact partial-stage timing, explicit circular progress lift,
stage-index/progress alias rejection, current provenance invalidation, distinct delay/connector
identity, wall blocking, dynamic-obstacle rejection, unavailable-vs-mismatched obstacle
observation, missing-input rejection and fingerprint mutation.

## Deletion/replacement audit

- `CanonicalExecutionRevalidation` remains the fresh same-callback summary used by the fresh
  Track/Cruise shadow path.
- Its builder now rejects a certificate decision which differs from the solved problem decision;
  it can no longer manufacture a retained candidate.
- Retained candidates can be built only by `build_canonical_retained_candidate()` after validating
  the sealed current-observation proof.
- No fallback, flag, timeout, lease, retry, parameter or final authority was added.

## Dynamic evidence

Automated single-car `make dev` run:

- Artifact: `output/20260823-011629/`
- Fresh Track/Cruise shadow solved and certified on most cycles; representative healthy windows
  were 41/41 certified.
- Natural numerical and physical rejects exercised the retained call site.  Retained telemetry was
  emitted in nine aggregate windows and thirteen status transitions.
- The largest retained window contained 38 attempts with a valid prior plan/cursor.  All rejected
  before candidate construction because the single-car run reported V2X `NoData`, not a fresh
  explicit empty observation.
- The runtime reason was initially collapsed into `invalid-input`; the adapter now reports
  `obstacle-observation-unavailable` separately from `dynamic-obstacle-present` and
  `obstacle-tube-identity-mismatch`.
- Fresh physical certification correctly rejected actual/current wall contact; no retained path
  bypassed that rejection.
- The published decision log remained `authority=legacy-normal-bypass`, while both fresh and
  retained shadow logs remained `selected=0`.
- One 26.159 ms callback overrun occurred during startup.  Later representative windows were below
  the 25 ms period and no retained publisher work was enabled.  This run does not establish a p99
  production timing gate.

This run proves fail-closed integration and publisher isolation.  It does not prove dynamic
retained acceptance, because no fresh explicit empty-V2X observation was available.  Gate B
production authority therefore remains pending.

### Explicit empty-world acceptance run

A second automated single-car run supplied an explicit empty
`v2x_msgs/msg/V2XVehiclePositionArray` with `frame_id=map` and the AWSIM `/clock` timestamp:

- Artifact: `output/20260823-014243/`
- The first two CLI-based attempts were rejected for test-input reasons, not product behavior:
  the first omitted `frame_id=map`; the second used host time instead of simulation time.  Old
  test publishers were found still running in the container and were terminated before accepting
  evidence.
- With one `/clock`-driven publisher, V2X diagnostics remained `health=Healthy`,
  `message_vehicles=0`, `message_invalid=0`, and source/receipt age approximately zero.
- At decision `20804`, the fresh five-state solution was correctly rejected as
  `certified-bound-violation` on stage-zero `virtual-progress-speed`: value `11.1254`, violation
  `0.0142602`, tolerance `0.0121254`.
- In that same cycle the prior plan passed current-world wall/empty-obstacle revalidation,
  canonical retained candidate construction, `RetainedCertified` selector admission and exact
  actuation extraction.  The outcome reported
  `retained=1/accepted/ready` and
  `retained candidate shadow-certified; publisher unchanged`.
- The aggregate retained window recorded `world=1`, `candidate=1`, `selector=1`, `actuation=1`.
- Unsafe retained windows also failed closed.  A separate current-wall-contact interval rejected
  the retained plan as `delay-prefix-blocked`; it was not allowed to inherit the old certificate.
- All retained and fresh canonical results stayed `authority=shadow, selected=0`.  Published
  control remained `authority=legacy-normal-bypass`.

This run closes the missing Gate B dynamic-acceptance evidence.  It does not authorize production
selection; connecting the selector to the final publisher and deleting the Track/Cruise legacy
fallback remain an explicit authority boundary.

## Rejected alternatives

- Reusing the old plan's physical certificate.
- Pairing retained stage `k` with current horizon stage `k`.
- Weakening canonical semantic residual admission.
- Adding a timeout, retry, lease or last-command fallback.
- Treating V2X `NoData` as an empty world.
