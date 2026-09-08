# Design and evidence boundaries

The original scene has two identical interaction sources: the final-authority
failure (S1) and the exact dynamic QP with actual warm start (S2). Their original
payloads remain unchanged; manifest.json identifies them by SHA-256.

The producer in mpc_controller_cpp.cpp fills semantic progress_stage_dt_sec
whenever progress metadata is available. It fills dynamics_stage_dt_sec from
those stages only when the older progress_contouring_active switch is true.
CurrentTargetTube currently consumes the latter vector. Frozen S1 has 20 input
stages of 0.25 seconds but target movement matches 0.025 seconds per stage.

First isolate sampling time with an observation-only comparison: reconstruct
the recorded affine prediction from its initial untruncated samples, validate
all original samples, and resample at cumulative semantic input times. Preserve
the existing one-second lateral interpolation and three-second truncation.
Do not reproject global motion into the finite wall-course window. Reseal the
candidate fingerprint. Observation/control-origin offset, acceleration and
projection differences remain separate hypotheses.

Only after a certified comparison, repair the production clock owner and add a
test exercising the same assembly helper. The obsolete legacy time-vector
consumer must disappear. No new normal authority or fallback is permitted.

Safety remains owned by exact nonlinear wall, timed-world dynamic and terminal
Stop proofs. Solver rejection is not evidence of physical infeasibility.
