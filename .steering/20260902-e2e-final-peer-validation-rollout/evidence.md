# Evidence

## Validation-only run

- run: `output/20260902-e2e-final-speed-committed-validation-v1`;
- world: unchanged deterministic four-peer `e2e-final` world;
- runtime mode: `speed_committed_teacher` in domains 1 through 4;
- base checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`;
- strict competition report SHA-256:
  `3b575aa9a0f5b75629f379ed3cd1cd6167d3fa2d4a9c76c7dc116e2eb166697f`.

All domains completed six laps with zero crash, wall or over penalties and no
post-start or positive-acceleration stall.

| Domain | Total / average / best lap [s] | Mean / max speed [m/s] | Stall [s] |
|---|---:|---:|---:|
| d1 | 396.677 / 66.113 / 62.279 | 4.711 / 6.642 | 0 / 0 |
| d2 | 386.302 / 64.384 / 62.444 | 4.876 / 6.538 | 0 / 0 |
| d3 | 391.243 / 65.207 / 61.859 | 4.722 / 6.367 | 0 / 0 |
| d4 | 383.448 / 63.908 / 62.294 | 4.785 / 6.411 | 0 / 0 |

Artifact SHA-256 values:

- result summary:
  `aa0cf3fc34ab3cb6baf185f2e3191faa19d58efd46fbccf48d42d672cb388c59`;
- d1/d2/d3/d4 motion analyses:
  `02ce614151636a5abcf5117127b2bed98b90b81f83323de85457ce79db1c230d`,
  `5b055523132736c81fbd67c2e71211110f4ccad369677f89a67b4a82e3fc8c4e`,
  `878654c125684f8c342e4edf5dd0f396dca55141701f28c148b5fa905a765309`,
  `dcd99568756550f52abe9d1f1e5679e5e67ccba08dc1f2cb9e163a007787dc93`.

## Admission decision

Admit the run as validation-only executed-teacher evidence.  It is an
independent execution and therefore detects run-specific regressions, but it
uses the same deterministic world as the training peer run.  It is not proof
of unseen-world generalization and must not be added to the training split.

As in the first run, AWSIM reached `FinishALL` and emitted every result JSON,
but the vehicle orchestrators did not enter post-processing.  `make down` was
issued only after every result artifact existed; all four MCAP files finalized
cleanly and passed the strict analyzers.  This is a separate lifecycle defect,
not a teacher-policy failure.
