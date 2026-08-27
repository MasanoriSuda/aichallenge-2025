# Audit: Stop-to-normal atomic authority handoff

## Observed symptom

In `output/20260827-211306/d1/autoware.log`, decision 1088 published an
explicit Stop after `Follow -> SafetyBrake`. At decision 1093 SafetyBrake
released to Follow, but the asynchronous Follow artifact was not yet ready.
Atomic admission compared it with the last *normal* intent, Cruise, rather
than the Stop that actually owned the wire, and emitted a generic Emergency
until Follow joined at decision 1094.

## Root cause

`last_published_canonical_intent_` had two incompatible responsibilities:

- retain the interrupted normal semantic for Stop shadow planning; and
- identify the effective command authority most recently serialized.

The first responsibility intentionally excludes Stop. The second must include
it. Reusing the normal-only ledger at atomic admission made the previous wire
owner invisible exactly at the Stop-to-normal boundary.

## Repair

- Preserve `last_published_canonical_intent_` as the normal-only semantic
  ledger used by Stop shadow selection.
- Add `last_published_authority_intent_` as the effective wire-authority
  ledger and update it only after final command publication.
- Permit a proven, already-published Stop to remain the previous authority in
  `resolve_atomic_intent_admission()`.
- Publish the same explicit Stop until the proposed normal intent has
  current-world authority, then change authority atomically.
- Clear the authority ledger when Recovery, disabled control, or final wall
  hold overrides the canonical result.

No timeout, grace, solver retry, wall relaxation, speed adjustment, or legacy
normal fallback was added.

## Static verification

- `make autoware-build`: 25 packages succeeded.
- Focused contract test: Stop-retention case passed.
- Structural source contracts: 65 passed.
- Full `multi_purpose_mpc_ros` package test: 47/47 CTest targets, 1,988 tests,
  0 failures, 0 errors, 0 skipped.
- `colcon test-result --verbose`: test summary was clean; the command also
  reported an unrelated stale missing `build/joycon_contract_guard/package.xml`
  path from the workspace test-result scan.
- `git diff --check`: passed.

## Dynamic acceptance

The bounded `make dev2` run is `output/20260827-214537`.

Domain 1 reproduced the exact boundary:

- decision 3048: `Overtake -> SafetyBrake`, explicit Stop published;
- decision 3077: `SafetyBrake -> Follow`, semantic successor was ShiftOut;
- atomic admission recorded
  `previous=stop, proposed=shiftout, effective=stop`,
  `resolution=previous-retained`, `previous_joined=1`, and
  `previous_external_stop=1`;
- the final producer was
  `canonical-stop-emergency/published Stop retained until normal authority joins`.

The old failure signature—using stale Cruise as the previous authority and
creating a generic normal-authority Emergency at this boundary—did not occur.
This closes the atomic handoff defect.

## Separate residual exposed by the run

The proposed ShiftOut did not subsequently obtain authority. Rejections moved
from `steering-unreachable` to `progress-lift-rejected`, and an asynchronous
solve also reached maximum iterations with a large constraint residual. The
vehicle remained under the correctly retained Stop and later encountered wall
contact / stuck Recovery.

This is not evidence for weakening Stop retention. It shows a separate
upstream lifecycle defect: a SafetyBrake-interrupted Overtake Mission can keep
requesting a pre-Stop ShiftOut that cannot be connected from the actual
post-Stop state. The next Slice must decide, using current-world physical
proof, whether to rebuild a same-Mission connector or atomically abandon it to
a valid Follow/Cruise producer. It must not add an elapsed-time escape or
relax steering, wall, or solver limits.
