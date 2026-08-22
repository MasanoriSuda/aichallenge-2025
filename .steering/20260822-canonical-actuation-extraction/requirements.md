# Canonical actuation extraction

## Baseline

- Branch: `develop_july`
- Baseline commit: `1dc8ba1`
- Preserve `aichallenge/result-summary.json`.

## Missing contract

The canonical selector can accept a complete stored plan, but there is no pure contract that maps
its exact current cursor to the five-state actuation semantics. Reusing the legacy `[speed,
curvature]` tail would discard optimized acceleration and virtual progress speed and could repeat a
final stage.

## Required correction

- Extract the control at exactly `cursor.first_control_stage_index`.
- Use the corresponding stage `i + 1` predicted velocity as the speed target.
- Preserve acceleration, curvature and virtual-progress speed independently.
- Convert curvature to tire angle only from an explicit positive wheelbase.
- Reject invalid plan, cursor/plan mismatch, exhausted cursor and non-finite output.
- Compare the extracted shadow actuation with the direct primal proposal without publishing it.

## Exit gate

- Stage zero and retained later-stage extraction are deterministic.
- The final control stage is never clamped or repeated after exhaustion.
- Runtime remains `authority=shadow, selected=0`.
