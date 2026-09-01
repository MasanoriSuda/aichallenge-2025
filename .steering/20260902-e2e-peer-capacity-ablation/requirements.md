# Requirements

## Objective

Determine whether the frozen 64-unit recurrent adapter lacks capacity for the
broader final-peer teacher corpus.

## Constraints

- Change only recurrent hidden dimension: 64 to 512.
- Keep dataset, frozen base/spatial artifacts, loss weights, optimizer,
  sampling, deadband and speed-input policy equal to the rejected frozen
  peer candidate.
- Keep speed embedding dimension 16; scalar speed remains disabled.
- Seed 2033 remains the development boundary and seed 2035 remains the final
  unseen audit.
- Do not change a runtime package, launch file, authority or checkpoint.

## Definition of Done

- Manifest diff proves hidden dimension is the only effective model/training
  change.
- Candidate is compared with both the 64-unit peer candidate and the previous
  non-peer recurrent candidate.
- Conversion is allowed only if both validation worlds and independent normal
  gates pass without regression against the previous candidate.
