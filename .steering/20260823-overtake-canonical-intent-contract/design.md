# Design

## Contract change

The normal canonical intent domain becomes:

```text
Track, Cruise, Follow, ShiftOut, Pass, Return
```

`Unknown`, `Hold`, `Stop` and `Rejoin` remain unsupported. Stop is an explicit supervisor
authority; nominal Hold has no proven producer; Rejoin belongs to Recovery.

Target provenance is mandatory for:

```text
Follow, ShiftOut, Pass, Return
```

This requirement is applied consistently to problem-context completeness and current retained-plan
provenance. The existing exact-intent equality in candidate qualification remains unchanged, so a
retained ShiftOut plan cannot execute during Pass or Return.

## Runtime effect

The only new runtime consumer in this Slice is the telemetry-only Overtake fresh shadow. Existing
production Overtake still converts the five-state result to the compatibility layout. No canonical
plan is stored or selected for publication.

## Expected replay progression

Before:

```text
physical certificate accepted
-> canonical plan: unsupported-intent
```

After:

```text
physical certificate accepted
-> canonical plan/cursor/candidate/command
-> world prediction
-> complete shadow selection, or the next exact rejection reason
```

If another rejection appears, it becomes the root-cause input for a new bounded Slice. It is not
fixed in this contract change.
