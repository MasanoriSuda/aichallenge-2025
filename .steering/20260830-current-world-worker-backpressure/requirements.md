# Requirements

## Objective

Freeze `d6e5c8ae` and fix the lifecycle defect proven by failure snapshot
decision 7602 without changing production authority, solver tolerances,
clearance, Mission timing, fallback or Stop policy.

## Evidence

- Persistent arm A fails.
- Stateless current-world arm B and the matching production arm G certify the
  selected `+1` homotopy with the same seven-state SQP.
- The live normal worker took 70--109 ms while receiving requests at 25 ms.
  In the matching window 79 submissions replaced 68 pending jobs.
- The current-world snapshot at the first authority failure was therefore
  physically feasible but could be overwritten before it started. Subsequent
  snapshots were already conditioned on the externally published Stop.

Exit classification: `A fails, B succeeds` and `offline succeeds, live fails`;
this is a Mission/scheduling lifecycle defect, not physical infeasibility or a
single-SQP limitation.

## Required change

- Keep the existing one-running/one-pending bound.
- For the canonical seven-state normal producer only, preserve an already
  queued immutable snapshot instead of replacing it.
- When running and pending slots are full, reject the new submission without
  blocking the 40 Hz callback.
- Keep latest-replacement semantics for tactical/pre-entry workers whose
  context invalidation contract depends on it.
- Continue current-world proof and provenance validation before authority.

## Non-goals

- No submission interval, timeout, lease or grace period.
- No wall margin, vehicle clearance or solver parameter change.
- No new fallback or Mission resume rule.
- No synchronous solve in the control callback.
- No authority promotion or relaxation of exact proof.

