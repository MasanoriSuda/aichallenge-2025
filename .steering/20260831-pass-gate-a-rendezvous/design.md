# Design: Pass Gate-A rendezvous

## Causal chain

1. ShiftOut reaches its physical lateral completion boundary.
2. The current callback has no Pass proposal because Gate A solves on a
   latest-only background worker.
3. The controller keeps the ShiftOut phase and continues publishing certified
   ShiftOut artifacts.
4. A certified Pass proposal arrives in another callback, but the instantaneous
   lateral completion predicate is no longer true.
5. The proposal is discarded even though it was rebuilt and certified from
   the current world.
6. The target moves out of the encounter while the phase remains ShiftOut;
   `locked target stale or lost` is the downstream symptom.

## Repair

Represent the ShiftOut completion boundary as a monotonic fact scoped to the
active target, Mission generation and selected side. The fact carries no
trajectory, wall proof or validity duration. Once observed, the phase boundary
remains ready until that exact encounter leaves ShiftOut or changes homotopy.

Pass admission then requires both:

- the scoped completion boundary has been observed; and
- a complete Pass proposal has been freshly joined and certified against the
  current world in the current callback.

This is an asynchronous rendezvous between two causal producers, not retained
execution authority. The Pass trajectory continues to start from the current
measured state and all current-world/physical checks remain unchanged.

## Alternatives rejected

- Increasing target continuity timeout hides the downstream symptom.
- Retaining a Pass proposal reuses a stale world/certificate.
- Adding a measured-state hold or another grace window introduces a fallback
  instead of repairing producer/consumer synchronization.
- Entering Pass before ShiftOut completion weakens the tactical phase contract.
