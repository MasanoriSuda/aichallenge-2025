# Design

## Observed semantic collision

Historical run `output/20260822-031809` contains DynamicWait decisions such as:

```text
phase=FollowPrepare
action=dynamic-wait
lateral_owner=dynamic-wait-prefix
longitudinal_owner=pass-floor
speed_window=6.00/inf/6.00
command=6.06m/s
```

The current private `current_control_intent()` maps every `DynamicWait` action
to `ControlIntent::Hold`. Historical `dynamic-wait-hold` evidence also shows
`ego=5.48 m/s` with no longitudinal limit: "hold" means holding a lateral
Mission path, not holding zero progress. A future zero-progress Hold contract
would therefore turn either kind of valid execution into a stop and recreate
the undesired mid-pass speed collapse.

## Resolution

Add the original committed phase to `AuthorityRequest` and resolve intent from
the complete authority record:

```text
DynamicWait + forward prefix + origin ShiftOut -> ShiftOut
DynamicWait + forward prefix + origin Pass     -> Pass
DynamicWait + lateral hold + origin ShiftOut   -> ShiftOut
DynamicWait + lateral hold + origin Pass       -> Pass
DynamicWait + missing lateral/origin identity  -> Unknown
SafetyBrake                                    -> Stop
```

Unsupported origins fail closed as Unknown. They are not guessed from the
current `FollowPrepare` phase. `ControlIntent::Hold` is deliberately left
without a producer until a true longitudinal-hold state is identified and
specified.

The resolver lives beside the authority orchestrator so problem fingerprints,
diagnostics and future shadow admission consume one semantic source.

## Authority boundary

This change only corrects intent provenance. It does not make Hold/Stop or
overtake MPCC production-active and does not alter final command selection.
