# Slice 2b validation

## Failure-first evidence

The focused tests were added before their production APIs and failed because the following contracts
did not exist:

- effective progress geometry reconstruction and fingerprinting;
- typed five-state actuation extraction;
- finite signed boundary values remaining extractable for later physical certification.

After implementation, the focused tests and the full package suite pass.

## Static validation

- `make autoware-build`: passed, 25 packages.
- `ctest --output-on-failure` in `multi_purpose_mpc_ros`: 33/33 passed.
- `git diff --check`: passed.
- Runtime configuration and production authority were not changed.
- All Track/Cruise shadow logs remained `authority=shadow, selected=0`.

## Dynamic validation

Valid run: `output/20260822-132440`.

The manually stopped single-car run crossed the circular seam six times. The observed seam-to-seam
intervals after the first crossing were 38.926, 38.951, 39.001, 38.926 and 37.925 seconds. The run
was used as a contract test, not as a lap-time tuning result.

Aggregate shadow pipeline counts:

| Stage | Count |
|---|---:|
| eligible / metadata / build / attempt / solved / finite / constraint | 11,233 |
| typed actuation proposal / legacy prediction conversion / physical check | 11,233 |
| physically certified | 10,995 (97.88%) |
| warm start / reset | 11,225 / 8 |

The acceptance alternative is met: every remaining rejection was explicitly classified as a real
`solution hard wall contact` or `solution swept wall path collision`. The earlier seam-contract
failure disappeared:

- `solution heading unavailable`: 0;
- `actuation-proposal-reject`: 0;
- production adoption (`selected=1`): 0.

The zero-length duplicate circular waypoint was still observed and normalized to 0.005 m at the
same seam stages each lap. Unlike the baseline, solver identity, cumulative stage distance and
physical certification all consumed that same effective geometry.

## Runtime and actuation evidence

Shadow timings across 11,233 attempts:

- build average / maximum: 0.041 / 0.211 ms;
- solve average: 1.383 ms;
- worst one-second-window solve p95: 9.288 ms;
- worst solve p99 / maximum: 10.886 ms;
- certificate average / maximum: 2.116 / 6.461 ms;
- total shadow average: 3.559 ms;
- worst total p95: 11.646 ms;
- worst total p99 / maximum: 13.503 ms.

Production callback telemetry reported a weighted 6.220 ms average and 28.190 ms maximum, with two
isolated 25 ms budget overruns. This is not a Track/Cruise promotion pass: scheduling/load must be
addressed before a 40 Hz live authority can rely on the shadow path.

The decision-ID actuation join produced 10,984 matches and zero rejects. Average / maximum absolute
differences were:

- predicted stage-1 velocity versus final target speed: 2.501 / 10.987 m/s;
- optimized acceleration versus final acceleration: 0.0009 / 2.1350 m/s²;
- optimized steering versus final published steering: 0.0438 / 0.1649 rad.

This does not assert command equivalence. It proves that predicted velocity, optimized acceleration
and optimized steering remain separately observable through final arbitration, so a future authority
slice can define their post-processing contract without reusing a legacy speed slot.

## Remaining blockers

- 238 cycles failed a real wall/swept-path certificate. The seam mismatch no longer hides these
  path-quality failures.
- Two isolated callback overruns remain in the shadow-enabled run.
- Large final-command deltas show that production post-processing/authority semantics still need an
  explicit promotion contract.

No parameter tuning or authority promotion is justified by this slice alone.
