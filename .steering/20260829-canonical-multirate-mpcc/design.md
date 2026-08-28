# Design

## Control flow

```text
40 Hz world snapshot
  -> revalidate candidate / last published certified prefix
  -> publish exactly one canonical command
  -> serialize command and record its physical predecessor
  -> submit latest immutable successor problem to background worker

background worker (effective rate limited by solve duration)
  -> seven-state MPCC solve
  -> exact physical wall proof
  -> immutable certified-plan store
```

The worker can replace a pending, not-yet-started observation with the newest
one.  It cannot replace the currently published authority.  Publication stays
with the 40 Hz current-world proof and the last actually published artifact.

## Why this is not MPC/MPCC switching

Both fresh and retained commands are samples from the same seven-state MPCC
artifact contract.  `retained` means receding-prefix execution, not a second
controller.  The only non-MPCC command in normal operation is the external
Emergency Stop when no certified normal command exists.

## Root-cause repair retained from the direct experiment

The ROS command field serializes steering as float32.  At the configured
double-precision model limit, round-to-float can be a few nanoradians outside
the double bound.  The physical-state contract therefore accepts exactly the
float32 serialization envelope and projects it back to the configured model
limit.  Values beyond the next float32 value remain rejected.  This is a type
boundary correction, not a tunable tolerance.

## Rejected alternative

Same-cycle direct solve was useful as an A/B diagnostic, but is rejected as a
production design because it blocks the callback for several control periods.
