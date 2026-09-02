# Evidence

## Frozen inputs

- raw checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial adapter SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- single Gate:
  `output/20260902-e2e-submission-freeze-single/e2e-competition-analysis.json`
- mixed-peer Gate:
  `output/20260902-e2e-submission-freeze-peer-v2/d3/e2e-run-analysis.json`
- privileged oracle:
  `output/20260902-e2e-peer-speed-committed-teacher/future-occupancy-maneuver-audit.json`

The participant controller and both production artifacts are byte-identical to
the packaged-default single-vehicle acceptance.  Commits since that Gate add
only offline audit code, tests and steering evidence.

## Result

Generated report:

`output/20260902-e2e-submission-freeze-peer-v2/e2e-submission-readiness.json`

| Gate | Result |
|---|---|
| production artifact identity | pass |
| three-lap single vehicle | pass; 252.2303 s, penalty/stall 0 |
| deterministic mixed peer | fail; 54.914 s low-speed interval |
| future-occupancy privileged oracle | inconclusive; labels/runtime forbidden |

Final classification: `single-vehicle-candidate-only`.

This candidate may be used for the qualified single-vehicle video and as the
frozen baseline in the submission material.  It must not be described as a
qualified multi-vehicle avoidance policy.  No production parameter, launch
default, checkpoint or runtime authority was changed in this Slice.
