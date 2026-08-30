# Findings

## Frozen evidence

- Run: `output/20260831-051051`
- d1 tactical `Follow -> Overtake`: four transitions.
- d1 canonical ShiftOut/Pass/Return intents: zero.
- `Overtake entry commit accepted/rejected`: zero.
- `Rate-resolved intent-transition causal execution shadow`: zero.
- Later warning: `causal pre-entry homotopy unavailable`.

The later warning is not evidence that the earlier Overtake candidate was
infeasible.  It is emitted after Behavior has already returned to Follow.
The first lost lifecycle boundary is not observable in the frozen log and must
be measured before changing the architecture.

## Dynamic validation

Runs:

- `output/20260831-054522` (`make dev2`; no validated entry reproduced)
- `output/20260831-054822` (`make dev3`; validated tactical entry reproduced)

The dev3 run reproduced one log saying `Follow -> Overtake` with
`entry_owner=1`, but produced no `boundary=draft`, Gate A result or entry
commit.  The binary was verified to contain the new lifecycle strings.

The apparent transition was emitted inside the 5 Hz tactical snapshot worker,
which calls the same `evaluate_v2x_behavior()` logger as the live callback.
It was not a live production-state transition.  The live callback remained
Follow/Cruise and therefore never built the Gate A draft.

The tactical worker itself was not generally unavailable.  d1 reported:

- adopted/usable (`lease=none`): 38 telemetry windows
- target mismatch: 86 windows
- target provenance mismatch: 10 windows
- stale: 10 windows

At the reproduced entry, the worker found a validated Mission, while the live
callback shortly afterwards reported `tactical candidate generation owned by
async worker`, `entry_owner=0` and no candidate.  One adjacent result was
discarded after about 0.29 seconds because target lateral provenance changed by
about 0.86 m against the 0.40 m acceptance bound.

## Root-cause classification

This is upstream of Gate A solver, wall proof and publisher authority.

1. The async worker owns tactical candidate construction.
2. A worker snapshot can select and certify a homotopy.
3. The live callback imports only usable worker evidence, but new-entry Gate A
   accepts only a same-cycle `V2XBehaviorState::Overtake` complete Mission.
4. The worker's Overtake state is diagnostic and is not imported; the live
   state remains Follow/Cruise.
5. Consequently no causal Gate A draft exists, and canonical ShiftOut cannot
   start even in worker epochs where a selected homotopy exists.

Per the frozen comparison exit rules this is a scheduling/lifecycle defect,
not demonstrated physical infeasibility.  A stateless current-world direct
branch succeeded in the earlier seq2230 snapshot, while the persistent live
pipeline never reached entry authority.

The implementation fix must connect the worker-selected homotopy to causal
Gate A as a tactical hint only.  It must not import the worker's path as
production authority: the causal worker still rebuilds and re-certifies the
trajectory from the serialized current command and current world.
