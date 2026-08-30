# Requirements: async terminal failure snapshot

## Objective

Remove observation-only architecture snapshot serialization and filesystem I/O
from the 25 ms production control callback while preserving the diagnostic
artifact needed by offline A/B/C/D architecture comparison.

## Frozen evidence

- Baseline: `1904cb07`.
- Run: `output/20260831-010730/d1/autoware.log`.
- Decision 1380 spent 25.583 of 29.696 ms in failure snapshot recording.
- Decision 1894 spent 68.999 of 79.454 ms in failure snapshot recording.
- The selected retained proof and Stop-lattice/Stop-successor joins together
  were below 5 ms in those cycles.

## Constraints

- No production authority, solver, wall, clearance, Mission or Stop policy
  change.
- Snapshot recording remains observation-only.
- At most one recording job runs and one latest pending job is retained.
- The control callback never waits for YAML or wall-grid filesystem I/O.
- No detached thread and no unbounded queue.

## Definition of done

- Failure snapshot persistence runs on one owned bounded worker.
- Worker lifetime is owned by the live controller.
- Static tests pass.
- A bounded dev2 run records a terminal failure snapshot asynchronously.
- Production `failure_snapshot` runtime no longer contains 25--69 ms I/O.
