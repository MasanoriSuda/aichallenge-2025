# R68 source availability observation

2026-09-12. Baseline/rollback165de6ed. This is an observation slice, not a
candidate or authority repair. All M4-M6 acceptance remains open.

Standard r68 reaches Ready with clock/odom after a transient prelaunch monitor
force-on; this single success does not establish the previous graphics stall's
cause. D1 has61 consecutive positive packets (10.289999774..11.789999774,
1.5s), peak measured1.171545m/s. D2 has30 moving positive packets
(12.309999727..13.034999727), peak0.560831m/s; its190-packet longest whole-bag
sequence is during startup and must not be called moving acceptance. Both stop
without laps. D1's61-packet moving segment contains20 distinct wire steering
words, from0.39073694 to0.21234451rad, so a permanently frozen steering word is
not the explanation for that segment. Individual callback records4123/4096 have zero overruns, p99
6.268/7.213ms; no recorded final publication-window violations. Bag receipt is
not actual simulator application. Teardown and user JSON/DLL restoration verified.

R414 rebuilds exact captured source/current requests and actual histories:
- D1context839 source305/job836, D2context949 source434/job946 each rebuild
  the same fingerprint and three-positive programme. Current raw side is0,
  selected source side+1/-1; selecting that side alone exposes GeometryChanged.
  Original geometry changes10749198986430161601->15741568466272881844 and
  15741568466272881844->7297210173200458920. Selection correctly refuses;
  no context guard repair is justified by these facts.
- D1proof938 carries original job936/source423. Complete applied Stop rejects
  WallRejected at12.339999736. R415 traces four attempted candidate rejections
  to occupied cell476325, at12.355/12.390/12.340/12.340. It does not prove
  physical infeasibility or a wall algorithm defect. Existing job933 still
  executes its independently checked braking remainder through index31, then
  expires. These are different source programmes, not a contradictory proof.

Remaining unknown: why no new compatible source is supplied after rest. The
scheduled path silently returns on absent draft, incompatible Store entries,
missing/currently invalid appointments, rejected observed requests or a busy
worker. The old aggregate source telemetry is not emitted on this production
path. Frozen snapshots cannot distinguish these upstream boundaries; a new
candidate or acceptance exception would be speculative.

Add a bounded observation at the existing source-publication boundary, using
its existing status-log interval. Record draft rejection, scheduled-submit exit
stage/counts, predecessor binding, normal worker/mailbox/Store counters and
source/current identities. No additional solve, candidate, queue, retry, hold,
context selection, deadline, parameter or authority. Logging occurs after the
published command's guards, and live callback timing must still be checked.

Acceptance: full build/package checks; a committed standard run identifies the
first missing source boundary with decision/source/context and original logs.
Then repair only the demonstrated producer with a failing regression and the
required architecture comparison. Promotion/deletion boundary: these fields
remain diagnostic; after M4 source availability is established, retain only
useful lifecycle telemetry and remove duplicate temporary observation paths.

Implemented draft/submit/selector/appointment/observed-request/successor-stage
observations and existing worker/mailbox/Store counters. Context selection runs
once per candidate with identical guards. Worker/mailbox/Store snapshots are
separate reads; do not infer an atomic event association from a status line.
Build109 passes26 packages; tests95 passes2,585 records/66 groups, source105,
C5 161. [Evidence](r68-source-supply-evidence.json). Next is committed standard
r69 to observe missing source and measure the logger's actual callback cost.
