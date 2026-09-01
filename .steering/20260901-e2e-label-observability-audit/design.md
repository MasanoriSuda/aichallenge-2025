# Design

Build the exact adapter input vector for deterministic per-run samples:

```text
normalized frozen conv5 projection
+ synchronized wheel speed
+ embedded frozen-base steering
```

The reference distance is the nearest observation from a different admitted
normal run.  Its p50/p95 values describe natural variation between successful
normal runs without relying on adjacent frames from the same trajectory.

For each material teacher query, find the nearest production-normal state and
report how often that distance is below the normal cross-run p50/p95.  A
material teacher state inside this envelope has conflicting desired labels in
the observable input space: teacher correction versus normal zero correction.

Also report the reverse direction (normal query to material teacher state) and
the frozen v11 correction on both populations.  The tool is diagnostic only;
no threshold becomes a runtime gate.

To distinguish compression loss from ambiguity already present in the physical
observation, run the same audit on a deterministic geometry representation:

```text
50 angular bins x (minimum range, mean range)
+ synchronized wheel speed
+ embedded frozen-base steering
```

This representation is not a production candidate.  It preserves local range
geometry without the frozen random projection and is used only as a control.
The existing three-seed action-separability probe also evaluates it with the
same classifier protocol used for the production representation.

## Decision rule

- Conflict only in the exact adapter input means representation collapse is the
  primary defect.
- Conflict in both representations means the static observation and label
  contract are ambiguous; a new head or runtime threshold cannot resolve it.
- A replacement representation is eligible for training only if it improves
  balanced accuracy without increasing normal false-material actions and keeps
  frozen focus-tail and peer direction accuracy.
