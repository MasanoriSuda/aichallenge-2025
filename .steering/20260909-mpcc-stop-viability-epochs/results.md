# Stop viability and state epoch results

Baseline4b2474da. Complete actual387and930/931evidence comes from the preceding
observation slice's dev2, not a new solve. Exact peer-only substitution preserves
Accepted930/+0.007091597519957693m and rejected931/-0.0003133950625793247m.
Anchor-only also preserves outcomes; around1ns mapping difference is the actual
published0.05cursor versus accumulateddouble0.049999999, not the old4.4ms rewind.
Frozen931normal9arms and supportStop4arms all solver-reject; no physical
infeasibility certificate. Whole physical-scene feasibility stays Unknown.

Raw same-run Odometry matches every recorded prefix-start XY and current speed
exactly. 927/928sourceage0;930source9.944999777 vsnow9.959999777 (15ms);
931source9.984999776 vsnow9.989999776 (5ms). Both recorded actuator prefixes
last0.13000000000000078s, although source-to-control durations are0.14500000000000135
and0.1349999999999998s. The measurement-to-now movement was omitted.

Before-r1native call-sequence test:1passes/5fail, including unrelated curved
position midpoint quadrature discrepancy6.0933638135nm atzeroage. Before-r2
isolates exact straight displacement, curved yaw and time accounting without
increasing tolerances:2zero-age pass,4nonzero-age fail. At2m/s,5/15ms omit10/30mm
and curve yaw0.00075/0.00225rad. Actual new stamped observation helper preceding
the unchanged actuator prediction passes6/6. Entire real state/longitudinal
prediction source test now34/34, including invalid/backward/stale source guards;
104single-authority source contracts pass. This native test exercises actual
prediction/observer and application call sequence; it is not a ROS node test.

Existing constant-twist source-to-now counterfactual shifts93019.1495264588mm/
-0.00223549508926rad and9316.56491314611mm/-0.000598722990191rad. Both reject:
terminal-0.0008582967537344999/-0.002501689556343001m; independentStop
-0.00018131388198128207/-0.0027823281397825195m. This can expose earlier loss,
not rescue old acceptance. It assumes body speed/yaw held through observation
age and retains scalar historical progress diagnostically; no actual-motion,
production-authority or full-cause claim. Full original captured inputs remain.

Selected implementation is in design.md: explicit stamped motion observation,
current-to-control prefix starts at its estimated now pose; current proof pose
has separate ownership from raw wall/contact monitoring. Existing actor delay,
steering resolver, longitudinal observer filters/history, solver and hard limits
remain. Build/package/fresh single/dev2 and source-epoch matching pending.


First full build rejects the attempted three-scalar geometry_msgs::Pose2D
constructor in the controller. Native-only tests do not instantiate that ROS
message. Preserve failed source/patch/log understop-viability-build-r1; use the
existing copy-and-field-assignment style. This is a compile repair with no
prediction semantics change. Rebuild pending.


## Final build and runtime

Corrected `make autoware-build`:26packages,3min41s; setuptools deprecations only.
Package `colcon test` and package-local `colcon test-result`:60CTestgroups,
2381records,0errors/failures/skips,24.74s. No host pre-commit; source104,
Python AST/local links and diff checks used. No further production edits.

Single-r1:6laps254.0740509033203s,penalty0,no observed moving override or causal
observation warning.258callbackwindows/10459cycles,max15.327ms,0overruns.
Commands10452/~39.99983281Hz/receiptmax33.73026848ms;Odometry13065/~50.00026459Hz/
43.62487793ms;clock52259/~200.00588981Hz/22.46356010ms. All three have no receipt
or source gaps>50ms and no source duplicate/backward in this run. No throttled
active-contract trace; do not turn its absence into all-cycle proof.

Dev2-r1: first moving D1Emergency930 at1.362512395068054m/s, wall
1788931444.348977206; no finish/details. D1/D2ninewindows/368/365cycles,
max20.969/23.696ms,0overruns;no causal-observation warnings. Both commands371,
40.03201029/40.00252817Hz, receiptmax40.68040848/33.34569931ms,0gaps>50ms;
source duplicates/backward12each, gaps>50ms2/1,max99.999998ms. Odometry453/454,
receiptmax303.46059799/307.26170540ms,gaps>50ms2/3;source0duplicate/backward/gaps.
Clock1817/1816,receiptmax302.45590210/300.52518845ms,gaps1/2;source0duplicate/
backward/gaps. Delivery continuity remains open. All runs stopped and protected
JSON restored to original hashes. Single acceptance passes; coupled rejects.

Actual runtime estimator matches:927sourceage19.999999ms,raw-to-now25.892985mm;
92815ms/19.419744mm;9305ms/6.812562mm. Each uniquely matches source-to-now
constant-twist projection with exactly zero XY and yaw error. This validates
implementation and timestamps, not true motion or independent steering latency.

New normal actual374/927Accepted ->928rejected after15ms:
terminal+0.0012454310131579938 ->-0.001161433927486577m. Peer-only and anchor-only
substitution preserve both outcomes. 928independentStop accepts
+0.007712347079671167m and derived386joins. At929the actual publication returns
to374; at930the captured final inspected plan is386. Keep these roles distinct.
The prior accepted929Request/374 is present but correctly marked invalid as a
same-artifact predecessor of386. Original374wall/dynamic proof passes, horizon
ends3.5364286976975055m/s, not rest.

Final930worldfp157141478484599072: complete inspected386physical proof replays
exactly without solves; original Stop terminal speed3.469446951953614e-18m/s.
Current Request has control speed1.3559838284616355 vs386expected1.2296352535200266,
pose error0.027859782062357417m. Current wall continuation passes; terminal peer
rejects-0.0013934640174382285m, independentStop-0.0011199987734020755m.
Runtime also reports an independentStop attempt from actual374rejecting peer;
do not assume choosing386alone caused the failure or substitute its clock into374.

Next: preserve actual sent Stop/normal command transitions and compare them with
same-run actuator/velocity/steering observations and the declared application
phase. Previously observed Unity10Hz actuation phase remains unmeasured; a short
published command is not proof of actual plant application. Inspect actual374
current-world revalidation independently if needed. No gain/delay/hold/solver
change follows from this hypothesis. Full MPCC/intents/dev3/dev4/gates/submission
acceptance remains pending.
