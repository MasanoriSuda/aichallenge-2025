# Requirements

## Objective

Promote the fully qualified wheel-speed, base-conditioned spatial steering
adapter into the participant package without leaking development-only paths or
granting authority in teacher and MPC modes.

## Qualified evidence

- candidate SHA256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`;
- strict offline aggregate, held-out, peer and normal-anchor gates passed;
- shadow single-vehicle gate passed;
- authority single-vehicle gate passed without penalty or stall;
- NPC seeds 2026 and 2027 both completed three laps in first place without
  penalty or stall;
- runtime inference coverage was 100% with no stale sample, inference error or
  authority clipping in both NPC runs.

## Requirements

- package the exact qualified artifact below `aichallenge_submit/`;
- verify its immutable SHA256 before loading;
- enable it by default only for TinyLidarNet `fixed_lidar_brake` production;
- leave MPC and diagnostic teacher modes unchanged;
- keep the legacy residual path empty so there is one learned correction owner;
- preserve explicit `spatial_authority=false` as the bit-for-bit base-steering
  rollback;
- make production defaults independent of the ML workspace and host `.env`;
- verify the sealed submission archive and one packaged-default closed-loop run.

## Definition of Done

- source, installed and archived model hashes are identical;
- launch, runtime and package tests pass;
- `make autoware-build` passes;
- the submission archive has exactly one `aichallenge_submit/` root;
- packaged defaults complete the single-vehicle acceptance without penalty or
  stall;
- all evidence and the rollback command are documented.
