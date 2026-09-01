# Requirements

## Objective

Evaluate the frozen production candidate with the same competition acceptance
contract across deterministic single-vehicle, runtime-NPC, and four-peer worlds.

## Constraints

- candidate3 SHA-256 remains
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- runtime remains `fixed_lidar_brake`
- no teacher, residual, recurrent candidate, or parameter promotion
- every admitted run requires Finish, all required laps, zero penalties, no
  post-start stall, and matching runtime provenance
- a failed world narrows the next data/model slice; it does not authorize an
  immediate heuristic patch

## Definition of Done

- single, NPC, and peer conditions have immutable run IDs and analyzer output
- failures are classified by the earliest physical evidence available
- production candidate and packaged checkpoint hashes remain unchanged
- generated run artifacts remain outside git
