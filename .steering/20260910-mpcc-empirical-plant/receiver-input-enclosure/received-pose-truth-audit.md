# Frozen received reconstruction versus physical pose epoch

2026-09-13 JST. force-r3 documentation HEADb6c6d04f, controllerfe976b39/build121/
package108. No production observer/model change. FullM4–M6 remain incomplete.
[Manifest and payload hashes](received-pose-truth-evidence.json).

**Phase erratum R572/R575:** initial tire must use Before Vehicle update. The
original sealed End-phase tire scores below are historical and superseded;
[corrected scores and limitations](received-event-phase-audit.md). Body scores at
all16matches are unchanged. Interpolation also worsens worst tire error after correction.

The validated seven-call probe and received queues run together. R567 preserves
19604physics records,3849received commands and972applications for the sole vehicle.
The run reachesReady, continues normal sends through1148, then loses normal source
at stationary1149. No moving final override,Start/laps or accepted restart;120s
host cap. OriginalDLL/userJSON restored, runtime stopped. Probe timing is diagnostic.

The algorithm and [protocol](received-pose-bracket-protocol.md) hashes were checked
before launch and after analysis. R568 retains25unique captured decisions with35
serialized aliases.16match an actual physics sample at the exact pose timestamp;
9pre-Ready captures have no physical rows and are explicitly unscored. No speed/
score-based cohort selection. Raw source/steady receipt, both interpolation
endpoints, fraction, held channels and own-source-epoch sensor errors are saved.

For16matched cases, mean absolute errors are:

| Component | Original held | Frozen bracket |
|---|---:|---:|
| Forward speed,m/s |0.01177593|0.00589199|
| Lateral speed,m/s |0.00134563|0.00089558|
| Body yaw rate,rad/s |0.00351325|0.00188984|
| Physical tire,rad |0.00288125|0.00166727|

All7velocity-supported interpolations improve u/vy, but8supported IMU cases have
5improvements and3regressions. Unsupported cases remain held; worst uerror0.0382093
at decision1005 is unchanged. The same-epoch physical u there is−0.0061712while
held publicu is0.0320381. Decision883 also lacks an upper endpoint and hasuerror
0.030875. Interpolation is a useful reconstruction hypothesis, not a universal
state enclosure or complete source/controller repair. Physical rest/recoil and
sensor own-epoch errors remain visible.

R569 is supplementary on this already inspected run. Two-past-sample extrapolation
improvesu but worsens meanvy/r. Using it only where the frozen bracket lacks upper
support lowersuMAE further, yet worstvy becomes0.0110603versus0.0054376original,
and worsttire0.0079151versus0.0073142. Blanket extrapolation is rejected. Do not
select a post-hoc mixture and call it independently validated or loosen proof
bounds around its errors.

Next compare a causal coupled observer initialized at a common received body
sensor epoch, then advancing the existing model and inserting subsequent received
measurements at their actual source epochs. Preserve raw inputs and derived state
separately. Exact same-run public command history and finite reset/rest coverage
are required; no private future/contact inputs, arbitrary zero seed or new runtime
fallback. This remains offline until causal/regression and coherent identity/proof
conditions hold. [Next bounded comparison design](received-body-event-observer-design.md).
