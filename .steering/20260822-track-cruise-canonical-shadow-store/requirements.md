# Track/Cruise canonical shadow store

## Baseline

- Branch: `develop_july`
- Baseline commit: `d5f6aec`
- Preserve `aichallenge/result-summary.json`.

## Missing integration

The certified Track/Cruise five-state shadow solve currently ends as a one-stage actuation proposal
and legacy-comparison telemetry. The complete canonical execution plan and its provenance are not
retained, so Slice 3 authority cannot later distinguish a fresh full plan from a warm start or a
flattened legacy vector.

## Required correction

- Build one certified solution identity from the exact shadow problem and physical certificate.
- Extract the full five-state/three-input plan directly from the primal.
- Atomically replace the canonical shadow plan store using a monotonic plan identity.
- Clear the active snapshot on control-history reset without resetting the monotonic high-water.
- Log extraction and store acceptance separately from solver and wall certification.
- Keep `authority=shadow, selected=0`; do not read the stored plan for command publication.

## Exit gate

- Every `status=certified` Track/Cruise shadow cycle has an accepted canonical store replacement.
- A store/extraction reject is explicit and cannot be reported as certified.
- No final command source, speed, steering or configuration changes.
