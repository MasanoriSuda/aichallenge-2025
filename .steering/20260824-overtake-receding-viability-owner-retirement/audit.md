# Root-cause audit

## Causal chain

```text
tight retained lateral corridor / async plan gap
-> canonical current-world proof withdraws normal authority
-> explicit Emergency owns the command
-> fresh DP/reference generation continues
-> legacy receding profile validator independently rejects its approximation
-> legacy code invalidates Mission generation
-> async canonical identity is destroyed
-> DynamicWait has no authority
-> Recovery
```

The canonical rejection is a correct fail-closed command decision. The later
legacy Mission invalidation is a separate normal-control authority and is the
architectural defect addressed here.

## Existing patch interaction

The previous Slice deleted downstream node-level wall/exit command handoffs.
This remaining branch is earlier: it changes Overtake FSM state inside
`update_overtake_line()` before the canonical producer can publish a newly
certified replacement.

The receding calculation also supplies a lateral reference and stage corridor
to the five-state problem. That producer role cannot be deleted in this Slice;
only its authority to terminate the canonical Mission is retired.

## Safety boundary

Demotion is allowed only if all of the following are true:

- phase is canonical `ShiftOut`, `Pass`, or `Return`;
- the lateral reference, path distances and wall corridor are complete;
- no current physical wall contact or wall observation failure exists;
- no front Emergency, solver-Recovery or forbidden-waypoint supervisor is
  active.

Outside that boundary the current fail-closed path remains unchanged.
