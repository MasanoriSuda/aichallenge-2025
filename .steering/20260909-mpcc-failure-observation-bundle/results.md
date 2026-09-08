# Atomic observation slice result

The observation defect is fixed and validated. MPCC completion is NOT achieved.
Baseline/rollback6c4875ed4d250a44cd813935b234863e9c184701. Production changes are
limited to the bounded recorder transaction and its two call sites. No new
authority, Store/candidate selection, solver, model, command or runtime tuning.

Commands: native pre-fix deduplication reproduction (expected failure), focused
architecture-snapshot14tests, scoped source-contract104tests, make autoware-build
(25packages), colcon test --packages-select multi_purpose_mpc_ros and scoped
colcon test-result (60CTestgroups,2362records,0failures/errors/skips). Initial
fixture rejection and host pytest plugin collection failure are preserved.

Run output/20260909-failure-bundle-dev2-r1 stops at moving authority loss.
D2decision1629, observation27.939999375, control28.069999375, worldfingerprint
6851073282052209746, actual published source1100, publicationdecision1623.
publication_bundle.status=present; original artifact, semantic initial state,
independent source wall grid and exact publication/failure joins validate.
extract_publication.py creates a derived replay input and seals the input hash;
it never alters the raw atomic record or substitutes a fresh solve.

replay_execution.py compiles an observation-only copy of the native retained
proof source. It adds a read of the already built complete stopping trajectory
and native dynamic rejection state, leaving every proof branch unchanged.
Original1100horizon has accepted static/dynamic proof (minimum3.335019m).
At current1629, delay and continuation remain clear, but both terminal choices
reject D1: normal reference1.56s/-0.006193101m; track1.555s/-0.006771093m.
Independent current-world Stop successor also rejects dynamic proof.
Default unused publisher-prefix/terminal-wall fields do not mean those stages
failed: terminal wall evaluation never runs after dynamic rejection.

The four Stop formulations solve on1629but all fail the same peer proof near
1.50409s. Normal A rejects linearization at19; B/C/D/G require a current front
target tube and cannot build one for this rear-peer-only Cruise scene. This is
not a bounded physical infeasibility certificate. The older4428comparison has
different failures and cannot explain this new scene; all outcomes are retained.

Held-out scoring does not establish a faulty constant-velocity producer. The
actual future includes D1's Emergency roughly0.73s after D2. Even that future
still overlaps the hypothetical complete stopping trajectory's unchanged full
peer circle near its end (-0.096/-0.103m). No future observation or assumed
cooperative braking may become current authority. The peer's observed clock is
already propagated to now by dynamic_world_observation; its freshness check
must not be confused with the subsequent CV future model.

Next structural question: can a rear-peer-aware candidate generator find a
complete certified stopping trajectory on the same sealed world where the two
fixed path references and front-target-only alternatives fail? Compare bounded
independent lateral stopping candidates with unchanged seven-state dynamics,
source pose/clock, full peer radius, wall reserve and actuation bounds. A
successful offline path is only a feasibility witness; production promotion
requires a solved canonical artifact, current proof and retired replaced path.
Also retain A's separate stage19linearization failure for causal inspection.

No six-lap dev2 result or submission acceptance. Same-run timing and restoration
are in validation-notes.md; all inputs/outputs in evidence.json. Original3834in
the earlier run is still missing, and this run does not replace that record.
