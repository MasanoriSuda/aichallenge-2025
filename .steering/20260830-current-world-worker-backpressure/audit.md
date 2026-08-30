# Audit

## Frozen failure

Run: `output/20260830-191617`

Snapshot:

`d1/mpcc_architecture_snapshots/000000007602-374a799d94e2b23e-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

### Causal chain

1. Sequence 6311 remained the executed ShiftOut artifact.
2. At decision 7602 its publisher prefix was wall/dynamic clear, but its
   retained track-reference Stop suffix intersected the wall.
3. The frozen current world had a fully certified selected-side stateless
   bundle; B and production G both accepted it.
4. Live worker cost exceeded the control period and 68 of 79 pending requests
   were replaced in the surrounding window.
5. The feasible decision-7602 world was not available as normal authority;
   external Stop became the serialized predecessor.
6. Stop and stale ShiftOut authority alternated, the physical state diverged,
   and the episode entered Recovery.

## Rejected hypotheses

- Wall proof clearance 0.20 versus planning clearance 0.40 is deliberate and
  was previously separated because 0.40 physical proof caused QP failure.
- The same seven-state SQP certifies B/G, so this is not an SQP limitation.
- Same-side B/G certifies exact wall, dynamic and terminal proof, so the frozen
  world is not physically infeasible.
- Changing Stop tracking or adding another grace/fallback would treat the
  downstream symptom while leaving the feasible world-loss mechanism intact.

## Acceptance evidence required

- Unit test proves preserve-pending submission never executes the replacement.
- Existing replace-latest test remains valid for other consumers.
- Package tests and source contracts pass.
- In dev2, canonical worker reports rejected/deferred submissions rather than
  pending replacement during expensive Overtake windows.
- No recurrence of Stop/normal alternation caused by a lost feasible bundle.

