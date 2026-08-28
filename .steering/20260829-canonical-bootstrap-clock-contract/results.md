# Results: canonical bootstrap clock contract

## Static verification

- `make autoware-build`: 25 packages passed.
- `multi_purpose_mpc_ros`: 52 CTest targets, 2,001 tests, zero failures,
  errors or skips.
- The deterministic retained test proves a bootstrap candidate starts at
  cursor `0.0` and produces an unchanged accepted current-world proof.
- Existing tests prove a moving time-aligned candidate consumes the elapsed
  suffix and a published plan advances from its atomic first-publish origin.
- The source contract proves `BootstrapCandidate` appears only at the normal
  Store consumer and only behind `executed_plan == nullptr`.

## Dynamic verification

Accepted run: `output/20260829-063134` (`make dev2`).

The immediately preceding run `output/20260829-062807` was discarded because
AWSIM never published `/awsim/state` or odometry; both controllers remained in
their expected missing-odometry startup failsafe.

In the accepted run:

- both domains obtained canonical Track/Cruise production authority;
- both domains left zero speed after the race start;
- d2 continued to publish Cruise authority with `available=80/80` or `81/81`;
- later retained transitions identify `clock:published-plan` with finite first
  publication origins;
- moving successor evidence remains `clock:time-aligned-candidate`.

This changes the frozen permanent cold-start
`steering-unreachable -> retained-proof-unavailable -> Emergency` loop into a
real bootstrap publication and subsequent published-plan execution.

## Separate failure exposed

d1 later lost normal authority after an intent transition. Its last executed
Follow artifact first failed progress continuity and eventually exhausted its
horizon while newer candidates were also exhausted. That is a moving
successor/horizon lifecycle failure, not a cold bootstrap failure. It remains a
separate frozen boundary and received no timeout, lease, fallback or parameter
patch in this Slice.

The current-world Overtake dynamic gate did not occur in this bounded run and
remains pending.
