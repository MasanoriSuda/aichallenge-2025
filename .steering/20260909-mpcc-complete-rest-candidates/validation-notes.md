# Validation notes

Baseline/rollback3c1cd03b. All new native/replay outputs under
output/20260909-complete-rest-*. No simulator was running during replay/build.
Single-r1began only after final package/native production replay.

- Exact odometry matching4264current pose gives speed8.718348824128569m/s,
  message stamp93.244997915s, receipt1788910126.4469273. Observation snapshot
 93.254997915s; current control origin93.384997915s. Current-time steering
 0.092016rad and previous-command age0.020s remain rounded log observations.
- Zero-solve actual3695original wall/dynamic proof passes, terminal speed
 11.014377145465252m/s. Current cursor0.569999987s, control speed8.709218141,
  expected9.512124290, pose mismatch0.314460147m. Delay/publisher interval clear;
  full continuation and both generated terminal paths reject wall. Independent
  Stop starts at the exact current pose and also rejects wall. No replaced
  artifact or physical infeasibility claim.
- Initial soft tangent overshoots immutable course by0.012779200122150058m.
  The old source-copy full A/B/C/D/G and free-rest reject before solve at19.
  Native13mm overshoot fails before. Shared initial/iterate tangent-domain
  selection passes all20adapter cases; same4264now progresses to wall-QP
  rejection. The original soft objective/reference/constraints remain intact.
- Source4264time horizon1.7806410245505548s is below even ideal braking
  time2.9030727136722247s (physical min-3m/s2), before solver boundary inset.
  Direct terminalv0on that horizon rejects acceleration rows. This says the
  fixed-time control problem cannot reach rest, not that the scene is infeasible.
- Eight retimed4264comparisons: fixed-max-law/free-controls x physical
  geometry-derived4.543438906s/source-maximum5s x depth0/3. All rejected:
  maximum-law exact wall contact; free-control wall QP. No budget/tolerance tune.
- Four retimed1629comparisons on5s: maximum-law0/3rear dynamic QP rejected;
  free0/3accepted wall, all-peer, actual rest and current command join. The
  distinct terminal/clock labels are sealed as new offline candidate contexts.
- Retimed helper's first build failed on std::string versus const char* artifact
  filename. Only helper fixed; failed build log remains in retimed/ and successful
  comparison in retimed-r2/. This is not a controller failure.
- New native production entry with rear+side peers fails old maximum-law
  candidate at dynamic clearance-0.0110606m. New entry passes; primary+secondary
  row count40for20stages. Candidate context differs from upstream owner;
  stale secondary, forged stage-clock and missing rest are rejected.
- First full package test exposed source profile length2.4m versus progress
  box5m. The free producer now intersects hard progress boxes with existing
  wall/course domains before solve. The original test remains and passes,
  as do all4focused CurrentWorldStop cases. It does not widen or clamp a proof.
- Initial full build25packages251s. Initial full test60groups had one native
  test failure (two colcon failure records); global2503record count also included
  unrelated cached packages. Final scope is explicitly MPCC only.
- Final make autoware-build:25packages16.1s. Final colcon test:60groups,
 2368MPCC-local records,0errors/failures/skips,24.9s. Native20adapter and
 47architecture comparison cases included. Source contract104cases unchanged.
  Build output contains setup.py deprecation warnings. No pre-commit pass claim.
- Final production replay1629: one direct candidate,35.444582ms cold offline,
  candidate fp3465481142038543833, problem fp17022304459249535851. Current-world
  join and command adapter accepted, full suffix wall/dynamic, min2.433887479m,
  terminalv-1.100496e-9m/s, firsta1.329595940m/s2. Upstream identity retained.
 4264still wall-QP rejected in125.019816ms. Neither replay publishes or establishes
  actual async/race acceptance. All runtime outcomes remain to be added.

Single-r1closed:6laps252.77877807617188s/penalty0, moving override0,
causal observation failures0.256reportedwindows/10375cycles,
callbackmax18.373ms/0overruns.10398commands40.001060827Hz,
receipt gapmax39.216995239ms,0gaps>50ms; source gapmax35ms,
no duplicates/backward/gaps>50ms. Odom50.002168Hz,receiptmax44.450283ms,
no gap>50ms. Previous pause/clock failures are not erased. No actual certified
Stop adoption in the single trace. All user JSON hashes restored by runner.
Dev2-r1started only after this run and its MCAP timing reader completed.
