# Design

## Hypotheses

| Hypothesis | Evidence for | Falsification |
|---|---|---|
| Production MPCC solve owns the tail | first run had a 1,025-iteration cold solve | overrun has small `mpc_ms` |
| Post-MPCC wall/authority work owns the tail | control callback contains synchronous footprint/corridor work | overrun has small `post_mpc_ms` |
| Recovery evaluation owns the tail | recovery safety can scan multiple poses | overrun has small `recovery_ms` |
| Publish/trace/marker work owns the tail | second overrun was not an obvious cold solve | overrun has small `publish_ms` |

No hypothesis currently has enough evidence for a behavioral repair.

## Timing ownership

Each callback carries a stack-local observation whose lifetime encloses the
scope-exit reporter:

```text
callback start
  pre_mpc_ms
  mpc_ms                 = MPC::get_control()
  post_mpc_ms            = authority/wall/final-speed preparation
  recovery_ms            = Recovery evaluation/arbitration
  publish_ms             = command publication, trace, marker/boost
callback end
```

The reporter also emits `unattributed_ms = total - known regions`, decision ID
and the last completed checkpoint. Early-return paths remain measurable and do
not require duplicate logging calls.

The observation is never passed to `MPC`, authority selection or command
publication. It has no configuration input and cannot change output.
