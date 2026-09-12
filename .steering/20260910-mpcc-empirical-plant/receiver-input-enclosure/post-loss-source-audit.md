# Post-loss source and scheduled wall evidence

2026-09-13, single-r9, baseline `b368ecab`, build123/package110.
Control/model `8d9810e0` unchanged. Runtime stopped, protected user JSON/DLL restored.
No Start/laps or accepted physical restart; all M4–M6 remain open.

R595/R596 validate actual moving normal1007 and first post-motion final1160
at17.089999618. It is followed by177normal publications. Last normal1342
(source1374/job1328/index13) is at21.674999515, before21.689999569deadline.
804normal sends,4275callbacks, maximum21.185262ms, zero25ms overruns/window violations.
R597public maximum1.059670448m/s; after1160 maximum0.068497255m/s.
Fixed7.9..21.7s source frequencies: velocity/tire28.571Hz,IMU20Hz,pose50Hz,
control39.884Hz,clock200Hz. This is the Ready motion/stop window, not race acceptance.

## Same-source failures

| Source / control / policy | Captured failure | R598/R599 classification |
|---|---|---|
|1081 /1177 /current steering|17.524999608,wall-refined QP maxiter4000,row497|Full-source exact rejection; original affine set reported infeasible by HiGHS|
|1080 /1177 /reference steering|same observation,exact wall sample347|Same wall contact reproduced,physical progress31.858 versus nominal reference46.618|
|1986 /1630 /reference steering|29.914999331,wall-refined QP maxiter4000,row495|Original affine set feasible,max row residual2.5964e-10; exact-QP warm/cold still reject,full cold source A certifies|
|2703 /1988 /current steering|40.094999103,exact wall sample61|Same wall contact reproduced at progress30.841,reference33.633|

1080/1081 share the physical/semantic input, with distinct initializer identities.
Rows495/497 are endpoint lateral/progress wall rows, not fixed initial equality.
A small primal residual does not authorize accepting maxiter. Exact-QP replay
uses a fresh numerical workspace; full-source reconstruction includes all earlier
solves and may differ from the captured persistent workspace. No solver reset,
iteration/tolerance/weight change is promoted. B/C/D/G target-dependent arms are
unsupported for this target-free scene, not proofs of physical infeasibility.

The native re-evaluation of1080 differs materially from its affine QP states
(e.g. around stage10). That is a solver/model consistency contributor, not by
itself the cause of later source1986 or the run's terminal loss. Reported nominal
path distance is not physical travel; physical progress plus lag reconstruct pose.

R600 setup discarded the required exact QP and produced no physical outcomes.
R601 corrects that setup with the same declared six candidates per unique world:
acceleration fractions1/4,1/2,1 and both original steering-prefix endpoints,
zero virtual speed. Physical-only source2703 explicitly uses a freshly assembled
diagnostic QP, not a falsely labelled captured matrix. All original physical
world/model/input limits and complete wall/dynamic/terminal proof are kept.
Negative steering at maximum inset acceleration gives accepted nonlinear forward
witnesses for all3worlds (final speeds1.3583/1.2801/1.2801m/s). These fixed-input
witnesses cannot execute as optimized normal control or bypass applied proof.

## Earlier failed invariant in the persistent stop

R603changes the working diagnosis: fresh certified sources keep arriving after
last normal1342, and Store advances to2700 around40s. The sustained22..40s loss
is not explained by absent source supply. At1343, new source1402/job1342 passes
source generation/context but fails current world/physical wall; old1374is
RestExpired. There is no exact1343snapshot, so this is log-level attribution.
370later scheduled proof failures include369WallRejected and1InvalidNominalProof.
Three recorded late reservations also expire; those timings remain separate.
Only later do the old artifact's prospective binding and then current envelope
construction fail. Do not replace the earlier wall boundary with the late generic
"terminal Stop course geometry unavailable" message.

R602/R604cross that boundary offline: cold-reconstructed1986 is wall-certified,
then evaluated at its own captured29.914999331observation using the actual prior
braking packet29.889999331 and the same-run static frame/policy. R602's inferred
floating delay fails the unique-nanosecond clock; R604uses saved config0.13s and
reproduces applied-program-unavailable / WallRejected at30.209999331.
It is a counterfactual fresh source evaluation, not exact live1343/source1402
replay or actual asynchronous completion/adoption. Original fingerprints match.

R605 point-observer compilation failed on a nonexistent encoder; R606 retains
that failed setup and records actual program fields directly. R606keeps R604's
verdict unchanged while checking four fixed native endpoint sequences from the
original per-step input population. This separates enclosure conservatism from
unsafe trajectories contained in the declared population; endpoints are neither
receiver truth nor an exhaustive proof. All four endpoint trajectories remain
clear for400steps. At the rejected interval30.209999331..30.214999331, the
enclosure includes positive yaw/heading variation although these four native
trajectories turn negatively or remain stopped. This is evidence to investigate
dependency loss in the enclosure; it is not proof that every interior input is safe.
Exact trajectories, ranges and unchanged rejection are preserved in R606.

Next: use this frozen scheduled boundary to repair a supported producer. Distinguish
nominal source feasibility, applied-program viability, fresh-body consistency and
actual deadline. Preserve unsafe wall/deadline/rest negatives, and require a
solved/certified common nine-state authority before any new live acceptance run.
No additional unchanged live observation or margin/timeout/fallback is justified.
[Evidence, outcomes, hashes and protected-artifact checks](post-loss-source-evidence.json).
