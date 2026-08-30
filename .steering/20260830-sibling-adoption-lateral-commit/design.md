# Design

## Causal flow before the change

```text
selected side reaches lateral clearance
  -> selected current-world solve transiently has no authority
  -> longitudinal-only pre-no-return remains true
  -> opposite same-epoch sibling is published
  -> tactical side mutates after ShiftOut completion
  -> full-track crossing replaces forward Pass
  -> lateral proof loss / progress stall / Mission expiry
```

The sibling Bundle itself is immutable and current-world certified.  The
defect is not certificate identity; it is that the tactical adoption contract
allows a solver-availability event to replace a homotopy that the vehicle has
already physically established.

## Contract change

Add `selected_homotopy_established` to the pure adoption request and its live
publisher state.  It is true only when:

- the locked target is currently laterally clear using the existing physical
  clearance observation; and
- the signed relative lateral position confirms ego is on the selected pass
  side.

The resolver returns `selected-homotopy-established` before considering the
sibling.  An accepted immutable token is also rejected at publication if the
homotopy becomes established before the command crosses the publisher.

This does not introduce a temporal latch.  It prevents fallback-driven
tactical mutation after the selected geometry has already become observable
in the current world.

## Non-goals

- deciding whether an explicit tactical replan should later cross the track;
- changing current-world candidate generation;
- hiding selected-branch solver failures;
- changing Pass speed, wall clearance or Recovery.
