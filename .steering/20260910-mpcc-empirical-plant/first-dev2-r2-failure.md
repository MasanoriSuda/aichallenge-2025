# Dev2 r2 first failure after terminal-time repair

Production commit746ec926 (`fix(mpcc): solve complete-rest feasibility over explicit
terminal clocks`). Buildr13:26packages; testsr9:2389records/0errors/failures/skips;
pre-commit passed. Canonical oldD1decision938replay accepts candidate7994973539003005584
in25.5021ms. No runtime success follows from that replay alone.

Fresh run `output/20260910-nine-state-dev2-r2` is rejected. First active moving
Emergency is D2decision991 atwall1789023130.605894858, now11.684999738,
control11.814999738, actualabout1.30m/s. World3671161868857863784
(hex32f29644e23b6e68). Teardown complete, originalDLL/userJSON restored by harness.
No simulator is running.

D2previously adopted a certified Stop atdecision858/source312 and subsequently
returned to ordinary Cruise. The failing inspected artifact is normal388, source
9.879999779, origin10.009999779. Its previous accepted Request is990at11.649999739;
current991at11.684999738. Both captured Requests use the same publication anchor
(first published control11.679999741, artifact elapsed1.6872479399767748).
A more recent Stop alternate455/source986is0.135s old and rejects its steering
join. Its exact artifact was not saved by this first-boundary recorder, so do not
claim an exact alternate455replay from a new solve.

Current-binary zero-solve replay:
`output/20260910-nine-state-dev2-r2-revalidation-r1`, compiled by
`output/20260910-nine-state-revalidation-tool-r5`.
990terminal clearance+0.0171142625m, independentStop+0.0175661449m.
991terminal−0.000805073174m, independentStop−0.000261108870m; wall/join pass.
New peer projected back to990makes it reject. Previous peer projected forward to
991still rejects. Preserving only the previous publication anchor does not change
991. This is not solely an async alternate steering-join failure.

Fresh canonical Stop solve at991 (`/tmp/mpcc-nine-state-dev2-r2-current-stop-r1.log`):
Both native-duration and maximum-support candidates solve, but exact dynamic proof
rejects atelapsed0.28s, clearances−0.001690/−0.001729m. Candidate fingerprints
17712756784084972518and10168496299262758443; measured solve110.97/122.59ms.
The short candidate's affine/nonlinear stage-node clearances reach−0.04479/−0.02177m.
QP feasibility does not establish full physical peer separation. Preserve all
hard constraints; compare a common dynamics/peer/wall relinearization on this new
sealed scene before another production patch. `dev2-r2-dynamic-sqp-r1` completed: OSQP short1/3iterations reject next dynamic QP;
maximum1iteration wall-rejects, maximum3iterations next dynamic QP rejects.
HiGHS short is Unknown and maximum rejects as QP-infeasible. No physical
infeasibility certificate. Nine native brake/power/steering laws also reject
first peer overlap atelapsed0.255–0.28s (`dev2-r2-native-witness-r1`).
Current991CA/CA1forecasts reject earlier, so changing only this late proof does
not restore feasibility (`dev2-r2-peer-causal-r1`).

Separate topic metrics completed (`dev2-r2-metrics/report.json`): D1/D2receipt
control40.1896/40.2011Hz; callback mean4.767/5.280ms, max24.698/33.117ms,
0/6overruns. D2decisions621/631/641/861/901are before first Emergency;
1191is after it. Publish/visualization and MPC/recovery checks explain the
measured regions; no overrun at991is recorded. Per-cycle percentiles were not
recorded in this run. New bounded sample telemetry is added with the peer fix.

Earlier precursor test: source388's own observed acceleration for1s thenCV
rejects the original ego path at2.0374029s; its CV prediction passed with
minimum0.3742811752m. See [causal prediction design](peer-prediction-design.md)
and the separate paired holdout. Production buildr14passes26packages;
regressionr10passes2392records. Integrated acceptance is still pending.

M4intent/actualStop-rest-restart/Recovery/Rejoin/Boost/async and M5–M6 acceptance
remain open. Continue authorized autonomous work and local commits; no push or
routine user confirmation. No physical tolerance/margin/backend/peer model change
is authorized by a diagnostic acceptance alone.
