# Task list

- [x] Accept the first-rate reachability dynamic Gate.
- [x] Isolate the remaining publication-time invariant.
- [x] Replace certified single-stage sampling with piecewise horizon sampling.
- [x] Add exact-boundary, multi-stage, horizon-end and physical-failure tests.
- [x] Add sampled-stage telemetry provenance.
- [x] Run build, full package tests and authority audit.
- [x] Commit the static Slice (`172e4d2`).
- [x] Run `make dev2` and confirm time rejects disappear.
- [x] Record the authority-migration decision.

## Definition of Done

- A publication interval within the certified horizon always resolves to the
  exact piecewise steering sample.
- Crossed semantic steering endpoints remain within the physical box.
- No stage timing, solver tolerance, physical limit or authority is changed.
- Runtime sample rejects are zero in the dynamic Gate or remain typed with a
  new upstream cause.
