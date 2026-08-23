# Task list

- [x] Audit current scalar Follow cap and five-state problem construction.
- [x] Add failure-first unit tests for all required observation cases.
- [x] Implement the pure stage-wise Follow contract.
- [x] Connect the contract to five-state problem construction in shadow only.
- [x] Add compact shadow telemetry with target/generation/reason.
- [x] Run focused tests and package build.
- [x] Run `make dev2` if the static gates pass and classify dynamic coverage.
- [x] Audit the diff for new authority/fallback/tuning paths.
- [x] Update evidence and commit without staging user-owned result data.

## Definition of Done

- Production publisher and existing Follow behavior are unchanged.
- Invalid/stale target states are rejected before QP assembly.
- Moving, stopped and opening-gap horizons are deterministic and finite.
- Follow target observation generation is present in the problem identity.
- No new config, timeout, lease, fallback or normal authority exists.

## Dynamic coverage disposition

The available `dev2` scenario exercised a retained Follow label without a
front target and a stopped-front `LowSpeedAvoidance` case. It did not produce a
moving-front Follow case. This Slice remains shadow-only; missing positive
dynamic Follow coverage explicitly blocks any later production promotion but
does not justify a synthetic fallback or intent rewrite here.
