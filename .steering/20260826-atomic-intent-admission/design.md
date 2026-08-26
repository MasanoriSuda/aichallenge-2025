# Design

## Root cause

`current_control_intent()` exposes the tactical proposal before its six-state
physical proof is complete.  The transition solver correctly rejects paths
that contact a stage wall, but the proposed intent has already displaced the
previous intent in the production call.  Revalidation of the previous plan is
then performed with the wrong intent and returns `intent-mismatch`; production
has no authority and emits Emergency.  The next callback often returns to
Follow and repeats the same proposal, creating Follow/ShiftOut/Emergency
chatter.

The issue is authority adoption order, not wall margin or solver strictness.

Dynamic falsification exposed a second, independent ownership defect.  Follow
revalidation re-projected the raw Cartesian V2X target after the current
six-state Follow problem had already built a continuity-constrained target
contract.  At a course crossing that duplicate projection could select another
branch and return `follow-target-observation-unavailable`, invalidating the
same Follow artifact that had just been formed from a coherent observation.
The observation owner, rather than the retained checker, must define the
course branch.

## Selected repair

Treat the tactical intent as a proposal until an atomic admission transaction
completes:

1. Revalidate ordinary candidate/executed plans for the proposed intent.
2. On an intent transition, synchronously solve and physically certify the
   exact proposal.
3. If it joins the current world, select it.
4. Otherwise revalidate the last actually published six-state intent and
   select it only when that proof remains current.
5. If neither side has proof, emit the existing explicit Emergency.

For a current Follow proposal, retained validation consumes the exact target
identity, generation, relative progress, speed and stage timing from the
current six-state Follow contract.  Only revalidation of an older published
Follow intent, when the proposed problem no longer owns a Follow contract,
may reconstruct evidence by joining the artifact target ID to the all-V2X
current-world snapshot.  That reconstruction remains fail-closed.

The selected effective intent is carried with the selected plan into command
identity.  The `last_published_canonical_intent_` ledger is updated only after
the exact serialized command has crossed the publisher boundary.

## Rejected alternatives

- Loosen the stage-wall certificate: admits the path that produced the fault.
- Add a ShiftOut cooldown: hides an authority-ordering defect with time state.
- Hold the last raw command: creates an uncertified normal fallback.
- Rebuild the old five-state MPC path: violates the single-authority goal.
- Always re-project the current Follow target from Cartesian V2X: duplicates
  course-branch ownership and reproduced the crossing regression.
