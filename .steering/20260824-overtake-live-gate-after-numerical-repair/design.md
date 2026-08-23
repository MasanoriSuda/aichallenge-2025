# Design

This is an observation-only vertical Slice. It reuses the existing typed
decision trace and fresh/retained canonical telemetry. The source, config and
authority graph stay frozen so the first live failure can be attributed to its
producer rather than to another mitigation.

The previous Gate at `output/20260824-005436` was causally invalid because
Track/Cruise lost normal authority before a usable Overtake interval. Commit
`8ff1a9e` repaired that upstream numerical contract; this rerun determines
whether Overtake is now continuously covered or exposes a separate defect.
