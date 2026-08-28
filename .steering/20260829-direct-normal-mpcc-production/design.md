# Design

## Production flow

```text
current world + current problem + last serialized command
  -> direct seven-state solve
  -> exact physical wall proof
  -> current-world / dynamic-obstacle proof
  -> publish certified command
  -> after serialization, mark that exact artifact executed
```

If the current direct solve cannot produce authority, the already existing
last-published certified artifact may be revalidated.  This is the standard
receding-horizon continuity path and not an additional controller or
uncertified fallback.

## Removed lifecycle

The following normal-control chain is deleted from production:

```text
publish command N
  -> bind problem N to command N
  -> enqueue async solve
  -> solve while later commands execute
  -> put unpublished candidate in store
  -> try to join it to a later predecessor/world
```

The pre-entry left/right tactical worker remains asynchronous.  It proposes a
homotopy and Gate-A evidence but does not own the normal command stream.

## Runtime policy

This slice does not tune update rate or solver settings.  Direct runtime is
measured first.  The A/B solve took about 81 ms end-to-end, so an effective
lower solve cadence may be required later; that decision must be based on
callback and command timing evidence, not a second authority path.
