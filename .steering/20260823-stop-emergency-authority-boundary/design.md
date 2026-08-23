# Stop emergency authority boundary design

## Current flow

```text
SafetyBrake
  -> canonical intent = Stop
  -> no Stop routing boundary
  -> legacy/progress normal solve
  -> zero speed / maximum deceleration post-processing
```

The observed final trace therefore reports `intent=stop` together with
`formulation=legacy-spatial-mpc-3state` and `authority=legacy-normal-bypass`.

## Target flow

```text
SafetyBrake
  -> canonical intent = Stop
  -> Stop emergency routing boundary
  -> canonical emergency supervisor
  -> zero speed + bounded held steering + maximum deceleration
  -> optional Recovery override
```

## Why this is not a Stop MPCC solve

There is currently no ordinary longitudinal Hold/Stop producer. The only proven Stop source is
SafetyBrake, which is already an emergency action. Creating a nominal Stop QP would add a second
normal owner without a real use case. The architecture explicitly retains an emergency stop outside
canonical normal MPCC authority, so this Slice deletes the accidental legacy solve instead.

If a future non-emergency Hold/Stop producer is introduced, it requires its own typed terminal and
progress contract plus shadow/dynamic proof. It must not reuse this emergency boundary.

## Structural change

1. Add a pure Stop routing decision that captures only `ControlIntent::Stop`.
2. Invalidate the async Follow context before returning from Stop.
3. Return the existing canonical emergency command before low-speed/direct and normal solver paths.
4. Include Stop in invalid-lateral-contract canonical emergency routing.
5. Preserve final recovery arbitration and execution-contract telemetry.

