# Follow canonical authority promotion design

## Observed data flow

```text
sealed Follow problem
  -> latest-only worker solve
  -> immutable canonical plan
  -> live target/wall/current-pose proof
  -> canonical command is built
  -> command payload is discarded
  -> legacy normal solver runs and owns output
```

The proof chain is complete before the discard. Re-solving or adding a fallback would preserve two
normal owners and recreate the authority ambiguity this migration is removing.

## Target data flow

```text
sealed Follow problem
  -> latest-only worker solve
  -> immutable canonical plan
  -> live current-world proof
  -> CanonicalNormalSelection
       command + problem + certified solution + plan + cursor + prediction
  -> shared canonical normal publication adapter
```

## Structural changes

1. Introduce one reusable selected-canonical payload used by Track/Cruise and Follow cycle results.
2. Make Follow retained/current-world validation populate that full payload instead of readiness
   booleans alone.
3. Generalize the existing Track/Cruise publication adapter to consume the selected payload and
   preserve exact plan/cursor-derived warm state.
4. Add a pure Follow production routing decision: non-Follow is unowned, coherent Follow with a
   complete selection publishes canonical authority, every other Follow state fails closed.
5. Return from `get_control()` at that boundary. This is the deletion of the Follow legacy owner;
   there is no fall-through fallback.
6. Rename/extend telemetry at the boundary so final logs state `intent=follow`,
   `authority=production` and whether a canonical command was selected.

## Fail-closed behavior

- The first worker-latency cycle may have no current-world-ready plan. It emits canonical emergency
  stop rather than borrowing a scalar command from another formulation.
- A disappearing/incoherent target while authority still says Follow is an upstream contract
  mismatch. It also fails closed until the supervisor changes intent to Cruise.
- Recovery remains allowed to override the returned canonical normal command in the existing final
  arbitration layer.

## Deletion boundary

The following sequence is forbidden after this Slice:

```text
Follow async evaluation
  -> no/ignored canonical command
  -> build/solve extended normal problem
  -> convert to legacy control
```

The shared solver code remains for later intents, but a Follow-owned cycle cannot reach it.

