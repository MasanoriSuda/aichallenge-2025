# Design

## Causal chain

The first imitation candidate learned the successful teacher runs well enough to
remove the original 203 m stall, but its own trajectory created a later NPC
geometry absent from the original dataset. It kept steering away from a left-side
return until the vehicle became physically pinned. Training longer on the same
two successful trajectories cannot add this missing state distribution.

## Relabel flow

```text
failed student bag
  -> raw LiDAR sequence
  -> detect confirmed contact-like return
  -> remove one-second pre-contact margin and embedded suffix
  -> current student checkpoint inference
  -> same runtime LidarGapTeacher
  -> keep active corrections only
  -> train-only sequence with immutable provenance
```

The source control topic is not read. Generated labels share the scan timestamp,
so synchronization delta is exactly zero. The successful seed 2027 validation run
is not relabeled or moved into training.
