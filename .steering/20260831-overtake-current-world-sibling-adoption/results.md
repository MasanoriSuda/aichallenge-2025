# Results

## Observed phenomenon

The predecessor run `output/20260831-110041/d1` reached a recoverable
immutable world at decision 2028 / sequence 1369:

- the live selected side `-1` failed exact physical wall proof;
- its same-epoch side `+1` sibling passed motion, wall, dynamic-obstacle and
  terminal Stop proof;
- an older side `-1` artifact remained executable for one retained command.

The consumer treated the last fact as if the selected homotopy were certified
in the latest world.  The valid sibling was therefore not evaluated until the
vehicle reached decision 2092, where persistent, stateless, rough and
multi-SQP arms all failed.

## Root cause and change

Retained command continuity and latest-epoch tactical feasibility used one
boolean authority meaning.  Sibling resolution was also reached only after
the retained candidate paths had already returned from the consumer.

The implementation now:

1. names and evaluates `selected_current_world_authority_available` only from
   the selected branch in the same immutable dual epoch;
2. inspects that epoch before older candidate, published and executed retained
   artifacts can return;
3. current-world revalidates the exact sibling Bundle;
4. serializes the sibling through the single canonical publisher; and
5. changes tactical side only after that exact command crosses the publication
   boundary.

An older retained command remains valid continuity evidence, but no longer
vetoes a newer certified sibling.  No resume rule, lease, grace period,
timeout, fallback, solver tolerance, clearance, weight or horizon changed.

## Static acceptance

- `make autoware-build`: 25 packages passed.
- package suite: 59/59 CTest targets, 2313 tests, zero errors/failures/skips.
- `git diff --check`: passed.
- Unit tests distinguish retained continuity from same-epoch selected proof.
- Source-contract tests require sibling resolution before retained candidate
  selection and require same-epoch selected evidence.

## Dynamic acceptance

`output/20260831-112206/d1` exercised a complete independent Overtake chain:

```text
Idle -> ShiftOut -> Pass -> Return -> Idle
```

No Recovery occurred in that episode.

`output/20260831-112650/d1` then exercised the repaired causal path:

```text
Published stateless sibling Bundle adopted:
target=d2, generation=1, side=-1->1, sequence=703, phase=ShiftOut
```

The command was published as canonical seven-state normal authority before
the tactical side changed.  This dynamically falsifies the former
`selected-authority-available` veto and closes this Slice.

## Residual failure kept separate

Immediately after the successful adoption, the legacy Mission candidate
generator reported `ShiftOut/Pass path requires wall clamp` and changed V2X
behavior from Overtake to Follow, while OvertakeLine and a current-world
seven-state Bundle still owned ShiftOut.  A newer side `+1` solved trajectory
was subsequently published, but the episode later lost its target and entered
Recovery.

This is not evidence that sibling adoption failed.  It is a new producer /
supervisor ownership contradiction: stateless current-world authority discards
the rejected frozen Mission geometry, while V2X behavior still treats the
absence of a replacement legacy Mission candidate as a reason to demote the
encounter.  The next Slice must freeze the earliest post-adoption failure and
compare persistent, stateless, rough and offline arms before changing code.

## Next acceptance

- A published stateless Overtake Bundle must not be invalidated solely because
  a legacy frozen-Mission candidate is absent.
- Target, homotopy, phase, current-world dynamic proof and terminal successor
  must still be current; wall/dynamic proof may not be relaxed.
- Behavior, line phase and canonical authority must have one observable owner
  after a stateless adoption.
- No new timeout, lease, grace, fallback or parameter adjustment is permitted.
