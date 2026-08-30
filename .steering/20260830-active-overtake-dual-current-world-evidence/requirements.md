# Requirements

## Objective

Classify the frozen Pass authority loss without changing production authority.
For active `ShiftOut` and `Pass`, evaluate the selected and opposite
homotopies from one immutable current-world snapshot with the unchanged
seven-state SQP and exact certificates.

## Constraints

- The selected execution side remains the only branch returned to the normal
  publisher.
- An opposite-side result is evidence only. It must not change Mission side,
  generation, no-return state, command, or publisher ownership.
- Both branch results must be tied to the same source sequence, decision,
  target observation, geometry, intent, and snapshot time.
- Do not change solver tolerances, wall/vehicle clearances, leases, grace
  periods, timeouts, fallback behavior, or Recovery behavior.
- A newer source atomically replaces both stored branch pointers, including an
  empty result, so evidence from different worlds cannot mix.

## Definition of done

- A unit-tested active-Overtake branch bank rejects wrong intent, side, epoch,
  and identity.
- The background worker evaluates both sides for active `ShiftOut`/`Pass` with
  separate solver contexts.
- The existing selected-side result is returned unchanged in authority terms.
- Live logs identify whether the selected side, opposite side, both, or neither
  produced an exact certified current-world Bundle before authority loss.
