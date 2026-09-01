# Design

## Authority boundary

The frozen raw and spatial policies continue to produce the canonical steering
command.  Once recurrent inference succeeds on the same verified Conv5 tensor,
the optional experiment applies:

```text
published = clip(spatial_production + clip(recurrent_delta, +/-0.24), limits)
```

The recurrent model owns no acceleration, safety-brake or watchdog behavior.
If its temporal input is unavailable, the spatial production command is still
publishable; no random state or retained correction is used.

## Configuration

Two additive parameters are propagated through the existing launch boundary:

- `model.recurrent_authority_enabled` (default `false`)
- `model.recurrent_authority_max_abs_correction_rad` (default `0.24`)

The host environment equivalents are accepted only with an explicit recurrent
checkpoint and expected SHA-256.  Supplying a checkpoint alone remains
shadow-only.

## Evidence

The preceding shadow run
`output/20260902-e2e-recurrent-shadow-shared` passed 3/3 laps with zero
penalties, 100% recurrent coverage, zero skip/error/reset and 8.005 ms weighted
mean inference time.  This slice changes only the final ownership of its
already-computed bounded correction and adds authority telemetry.
