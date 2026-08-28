# Requirements: Current-world A/B contract

## Objective

Make the frozen architecture comparison reach the common seven-state SQP on
the native Pass failure snapshot.  The stateless arm must be reconstructed
from `ReplayWorld`, not from target stages retained by the persistent Mission.

## Invariants

- Production authority, publisher, solver settings, costs and clearances do
  not change.
- A and B consume one immutable world/problem fingerprint.
- Target projection uses the same constant-velocity observation model as the
  exact dynamic proof.
- A short receding horizon is not rejected merely because Return or a complete
  stop is outside that horizon.
- Every accepted receding successor has physical braking authority and carries
  an explicit non-authoritative contingency Stop intent.
- Failure remains typed and fail-closed when the target, course window,
  generation or braking authority is unavailable.

## Definition of Done

- Failure-first tests prove that B no longer depends on Mission target stages.
- Persistent target-stage mutations cannot change the stateless seed.
- The frozen native snapshot reaches the SQP in A and both B arms.
- The result is documented before C/D or any production change is considered.

