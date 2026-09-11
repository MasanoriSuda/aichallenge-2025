# Exact publication clock endpoints

2026-09-11JST. Authorised MPCC completion; baseline/rollback2994c7b2
(codebb2d8843). Overall M4–M6 remains incomplete.

Earliest reproduced representation defect: r31D1decision602 has nominal ROS
1619999963ns and final1644999963ns, exactly25000000ns later. Converting the
nominal to binary64 and adding .025 gives1.6449999629999998429, while converting
the final integer stamp gives1.644999963000000065. The strict double guard
rejects that endpoint. This differs from the real30–35msROS advances inr37/r39;
fixing the endpoint cannot claim those runs pass.

The producer must declare which clock domain its input programme uses. Add an
optional, fingerprinted integer-nanosecond programme grid: first epoch, period
and maximum publication delay. Derive future nominal/latest epochs by checked
integer addition, then convert once for the numerical enclosure. Both the
admitted-input bounds and final publication guard use that same grid. Do not
add a tolerance, nextafter acceptance, wider25mswindow or clock-regression mask.
Legacy programmes retain their exact continuous-double semantics and hashes.

The current request API carries seconds. Bind the new grid only where all
input seconds round-trip uniquely to integer nanoseconds; never guess an
ambiguous stamp or round an arbitrary continuous time. The implementation must
make this supported range and fallback/rejection policy explicit before live
promotion. An explicit integer provenance from the ROS boundary is required
before claiming arbitrary large-epoch support. Remaining programmes must retain
the selected clock representation and reseal a new valid origin. Internal YAML
needs a distinct schema and must reject missing/mixed/tampered grid metadata.
No public ROS topic or evaluation schema change.

Required: frozen602negative reproduces with the old guard; exact endpoint passes
only for a correctly bound integer programme, +1ns and adjacent non-grid doubles
reject; negative/regressed clocks, invalid/overflow/non-grid origins reject.
Full input interval/rest/native coverage and hashes/YAML/Stop materialization
must agree with the same boundaries. Legacy exact endpoint tests remain intact.
Run focused regressions, make autoware-build, package tests, original negative
history/wall/peer replay, then fixed-code live validation. The implementation and local validation below do not establish integrated acceptance.


## Implemented and locally validated

`PublicationNanosecondClock` binds first_ns, interval_ns and maximum_delay_ns.
The seconds-only request API binds a grid when all three values round-trip
uniquely within2^51ns; other inputs retain their original continuous semantics.
This is an explicit representation distinction, not permission to round a
non-grid timestamp. A retained integer programme rejects an unsupported new
origin rather than silently reverting to a continuous clock. Arbitrary large
ROS epochs still need original integer provenance at the ROS boundary before
claiming exact ns support there.

`publication_epoch` uses checked integer multiplication/addition for that grid.
Programme construction, guaranteed input coverage, positive-packet expiry,
complete-rest floor, snapshot deadline and final guard share its endpoints.
Final clocks must themselves be grid-representable and nonregressing. No tolerance
or extra ns is admitted. The common input fingerprint includes all integer
metadata; materialized Stop uses internal applied-stop-provenance-v4. V1–V3
retain original semantics; mixed/missing/tampered grid/schema records reject.
No new normal authority is introduced or old failed physical case accepted.

Buildr65:26packages/4m59s. Testsr54:2466records/63groups,0errors/failures/skips,
32.0s. The original continuous nextafter boundary test remains. New tests cover
frozen602,10000integerorigins,1nsoverrun,non-gridadjacentdouble,regression,
overflow,hash/schema/Stopretention and bothclockmodes×18native schedules throughrest.
R225replays20finalfailureinputs: allphysicalproofsAccepted, exactly602nowpasses
thepublicationbracket; theother19realoverrunsremainRejected. R226–R228retainall
21previousphysical/Stopclassification results, including theoriginalr18historygap.
R223wasanisolatedcompiledeploymentfailure;R224SIMDprovedbitidentitybutdidnot
closecost,soisnotpromoted. [Sealed evidence](nanosecond-publication-evidence.json).

- [x] Implementation, legacy compatibility and exact frozen endpoint regression.
- [x] Build/package/native tests and current/old recorded proof replay.
- [x] Fixed committed dev2-r40 failed at a real publication overrun; see [scheduled publication work](planned-publication-design.md). Remaining M4–M6 stays open.
