# Design

## Earliest uncertain boundary

The first observed failure is not Recovery or Emergency Stop. It is the
transition from an already certified ShiftOut artifact to a Pass-entry
artifact whose exact current-side physical prefix is unavailable. Recovery
and speed loss are downstream effects.

The current log is sufficient to freeze the failure family but not sufficient
to decide whether the unavailable prefix was caused by retained Mission
geometry, candidate generation, the local SQP, or true physical geometry.
The architecture comparison therefore precedes production code changes.

## Snapshot handling

Architecture snapshots are currently written to the Autoware container's
relative `mpcc_architecture_snapshots/` directory. The comparison run will
copy the relevant sealed snapshot and wall grid to the run output before the
container is stopped. This is an observation workflow only; it does not alter
normal command authority.

## Exit mapping

- A fails, B succeeds: persistent Mission lifecycle defect.
- A/B fail, C succeeds: candidate generator defect.
- A/B/C fail, D succeeds: single-SQP/local-convexification limitation.
- Solve succeeds but exact proof fails: model/certificate mismatch.
- Offline succeeds while equivalent live solve fails: scheduling/lifecycle.
- All methods fail without a bounded certificate: `Unknown`.
- All methods fail with a bounded physical certificate: physical
  infeasibility.
