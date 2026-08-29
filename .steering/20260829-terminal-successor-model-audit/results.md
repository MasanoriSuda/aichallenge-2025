# Results

## Root cause

The retained terminal certificate and the actual Emergency Stop publisher did
not execute the same lateral controller:

1. the proof discarded the already-published steering-rate command during the
   unavoidable first publication interval;
2. it then held steering constant while braking;
3. production instead tracked the reference path while moving.

The resulting wall rejection described an unpublishable hypothetical path, not
the Stop path which subsequently owned the wire.

## Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 test targets,
  2096 tests, zero failures.
- Dynamic run: `output/20260829-172407` using `make dev2`.
- D1 terminal Stop proof transitions: 9 attempted, 9 certified, 0 failed.
- `terminal-contingency-unavailable`: 0 after race start.
- Example decision 1130:
  - normal continuation: current-stage prefix,
    `exact=invalid-lateral-bounds`;
  - terminal Stop: `model=none`, `exact=accepted`;
  - wall: valid and clear;
  - dynamic proof: clear;
  - steering hand-off/final: `-0.167745 / -0.274975 rad`.

## Remaining separate failure

At decisions 1464 and 2928, full continuation was rejected before a safe
current-stage prefix existed.  Terminal proof was therefore not attempted and
Emergency Stop remained correct fail-closed behavior.  The subsequent first
stage departure, wall interaction and Recovery are not evidence against this
Slice; they identify the next candidate/current-stage feasibility audit.
