# Design

## Root cause

The failure is not caused by insufficient acceleration or an overly short
solver horizon. The Pass optimizer correctly reported a target-bound conflict,
but the repair path then replaced that result with an old Mission-aligned
lateral trajectory. That trajectory was checked against the wall only.

`TargetBoundExecutionHoldRequest` made predicted-sweep proof conditional on
whether one particular solved-prefix source was used. A second old-path source
(the aligned receding-horizon warm start) therefore moved laterally without a
current opponent certificate. Even after the generic target sweep became
unsafe, Mission lifetime/progress rules kept the prefix authoritative.

The visible SafetyBrake was downstream: the unsafe prefix had already reduced
separation until the bodies overlapped.

## Repair

1. Target-bound repair always starts from a constant measured-lateral prefix.
2. Do not import the latest/last-feasible solved lateral trajectory or the
   aligned receding-horizon warm start into this repair path.
3. Make a valid and separated current target sweep mandatory for every held
   prefix. Recoverable side-contact continuation remains the only explicit
   exception and keeps its existing wall/front hard guards.
4. Remove the target-bound solved-prefix state and its two hold-budget config
   parameters. The general solved-path age remains in use by the ordinary
   current-world execution-source handoff and is not part of this deletion.

This narrows persistence to tactical identity and bookkeeping. Geometry is
rebuilt from the current world, matching the frozen architecture rule.

## Expected behavior

When a future target conflict appears:

- a fresh same-side current-world Mission may replace the old one;
- otherwise a short current-lateral prefix may bridge planning only while its
  current target sweep remains separated;
- when that sweep becomes unsafe, the hold loses authority immediately and
  the existing canonical Stop/supervisor path owns the unresolved interval.

No old lateral ramp survives solely because Pass is committed.
