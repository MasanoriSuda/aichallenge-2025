# Scheduled current evidence and physical Follow origin

Baseline `c057d6b8`; rollback is this commit. C4necessary gates are implemented
but remain disconnected from normal authority. Full adoption context and C5single
dispatch are pending; no new certificate may execute through an ordinary adapter.

## Invariants and implementation

- Fresh component samples are checked at their own source epochs in the ORIGINAL
  scheduled input tube. Repeated original raw components must be exactly unchanged;
  regressed or uncovered new epochs reject. New pose compares world x/y and yaw
  sin/cos; velocity, yaw rate and tire use their individual native intervals.
  The last issued physical steering must serialize exactly to actual history.
  This is consistency of observations, not a new initial population or proof cache.
- The publication ledger must match both original and fresh raw histories and every
  declared intervening packet, source identity, index, nominal epoch and raw clock
  bracket. Missing Stop, unexpected sends, reset and publication overrun reject.
- Fresh world recheck retains original swept body and rigid corners through rest.
  Partial intervals begin at max(original begin, fresh now), retaining the complete
  swept range conservatively. Peer query durations cannot be negative. Current
  wall/peers/Follow are checked with unchanged footprint, limits, policy and profile.
- Fresh Follow uses physical control-origin progress. Every possible nearest source
  course branch must agree within the existing continuity condition; circular lap
  association is explicit. The forecast at elapsed zero must equal current gap.
  This coordinate association is not a proof of a fresh predicted population.

## Reproduced producer defect and removal

Both live Follow producers used a gap already measured from the physical control
pose and added its waypoint-relative lag again. Consumers then added the complete
physical control-origin progress, double-counting that lag. R259reproduced absolute
peer-position error equal to ego offset (-.1, .05, .2m), independently of elapsed
forecast time. New PhysicalOriginFollowTargetBuildRequest excludes waypoint lag;
both canonical Follow and Cartesian rejoin producers use it. The obsolete second
waypoint projection in Cartesian rejoin is removed. General offset-form builder
and old public layouts remain compatible; internal consumers use the explicit type.

R261shows a physical-certificate consequence in the unchanged straight fixture:
with a stationary target at gap3.7m and hard gap3m, erroneous+.8m offset accepted
(minimum3.0261700066378263m), while corrected origin rejects terminal contingency.
At corrected gap4.7m the complete applied Stop still accepts. This is not a
physical infeasibility theorem or a runtime collision claim.

## Tests and limits

- R256107tests and r257108tests pass mixed source epochs and rotated/wrapped poses.
- R258110tests pass complete actual prefix and negative history/clock/source cases.
- R259reproduces the producer offset; r260111tests pass the new physical-origin API.
- R261focused full Stop comparison establishes acceptance impact above.
- R262114/115: new wall fixture used the wrong map Y-axis, placing its obstacle away
  from the intended pose. R263uses OccupancyGrid.world_to_grid and clears the derived
  index;115/115 pass. Production collision arithmetic is unchanged.
- R264reproduces acceptance of an offset-form fresh Follow forecast at the new C4
  boundary. R265adds exact gap/forecast-origin binding; 115/115pass (617ms).
- Buildr75passes26packages; testsr63first rejected one old source-name assertion
  (reported also by its CTestwrapper). The existing boundary test now requires
  both explicit physical-origin calls and excludes the old builder. Testsr64passes
  all2505records/64groups in33.1s; no C++source or safety condition changed for this.
- Live Follow validation, immutable adoption context, dispatch timing and M4–M6
  remain outstanding. The runtime findings below do not close them.

Changed code: scheduled header; retained header/source for physical-origin builder
and required prefix check; applied source for three necessary gates; node Follow
producers; retained regression tests. No rate, delay, model, bound, tolerance,
margin, timeout, solver budget or hold/grace change. Existing synchronous normal
publication is still the only normal authority and must be retired with C5.

See [next integration obligations](scheduled-context-dispatch-design.md) and
[tasklist](../tasklist.md). Follow runtime findings remain distinct from proof API
unit tests and final multi-vehicle acceptance.

Final native r265:115/115pass(617ms). Buildr75:26packages/4m37s;
testsr64:2505records/64groups,0errors/failures/skips.
Runtime dev2-r44: `first authority/publication-window failure; preserve earliest failure`. See the sealed run for exact scope;
this statement is not Follow coverage or race acceptance.
[182sealed files](scheduled-current-evidence.json).


## Dev2-r44 runtime result (failed; Follow not covered)

Baselinec057d6b8plus the preserved working patch; built libraries match buildr75.
D1first moving override: decision892, wall1789120695.299804314, Ready phase,
Cruise, actual0.21m/s. Snapshot preserves nominal9.239999793, rawbefore9.274999792,
deadline9.264999793: actual publication is already9.999999ms late. Exact wire
packet matches the certificate; this is not an identity mismatch. D1decision905
subsequently publishes within its before limit9.589999785 but returns9.594999785,
4.999999ms after deadline9.589999786. D2first moving override1070similarly has
nominal14.239999681/before14.269999681/deadline14.264999681(5ms late).

No Follow authority records or completed laps. This run does not exercise the
changed live Follow projection and cannot establish its dynamic acceptance.
The same synchronous publication timing family remains unresolved; C4unused APIs
are not a repair for it. Stop trace counts do not prove multi-tickStop/rest/restart.
Shutdown starts at wall1789120702.7041407; the later D2/D1epoch-mismatch traces at
1789120703.575852033/1789120704.126242183are downstream of teardown and not the
first cause. Do not mix them into a new pre-shutdown controller regression.

Public MCAPcommand receipt averages37.93/38.22Hz across the recorded intervals;
maximum gaps164.25/107.27ms. Invalid commands and actuation identity rejection
counts are0. Callback99/53overruns and54/25adjacent pairs include startup/teardown;
these aggregates are not phase-specific race metrics. Exact moving deadline
failures above independently reject timing acceptance. No margin/window relaxation.
All runtime containers removed; original DLL and both userJSONhashes restored.
