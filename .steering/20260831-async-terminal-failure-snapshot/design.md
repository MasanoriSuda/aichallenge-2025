# Design: async terminal failure snapshot

## Root cause

The terminal failure recorder is diagnostic, but it builds a replay artifact
and then serializes a large YAML and wall-grid file synchronously before the
normal command returns.  That reverses ownership: observability can invalidate
the authority interval it is observing.

## Change

Keep the current immutable evidence capture and detail construction.  Move
`record_proof_failure()` and all filesystem persistence to an owned
`LatestOnlyWorker`:

```text
control callback
  -> detect terminal proof failure
  -> capture immutable replay snapshot
  -> submit latest observation job
  -> return normal authority

observation worker
  -> serialize YAML
  -> persist wall grid
  -> log completion/error
```

One running plus one latest pending job bounds memory and prevents repeated
failures from creating an unbounded write backlog.  Supersession may discard
only an unstarted diagnostic snapshot; it cannot change control authority.

## Non-goals

- Do not optimize or relax retained physical proof.
- Do not change when a failure is classified.
- Do not make snapshot completion an admission condition.
