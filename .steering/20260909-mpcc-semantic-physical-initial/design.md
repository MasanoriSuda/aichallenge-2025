# Physical integration begins at the immutable semantic initial state

Autonomous repair scope,2026-09-09. Baseline5cfc50bcplus the separately validated
peer-body slice. No build/run/replay is active at the start of this slice.

## Evidence and invariant

The accepted QP equality residual does not change the physical initial pose.
D1source396from`output/20260909-peer-envelope-dev2-r1` has semantictheta0and
accepted raw primaltheta−2.1804138714481612e−9m. Its exact course window begins
at the origin. Native `evaluate_temporal_frenet_transition` with the recorded
first control rejects the raw initial state; the identical function/control
accepts the semantic initial state. The comparison is in
`world-target-comparison-r2/396/report.yaml`. The source fingerprint is
9043314919295532327; the world-target candidate has a different seal and is
only an offline comparison input.

The producer is `build_execution_artifact` in `mpcc_rate_resolved_shadow.cpp`:
it copies raw QPstate0into the artifact, then physical/retained/connector
consumers treat it as the actual initial pose. The numerical certificate is
valid; the physical course-support invariant breaks after that boundary.
Architecture replay already starts from semanticx0, explaining its different
stage0outcome. The downstream Emergency/Recovery masks missing normal
authority. This defect is distinct from the omitted side-peer constraint and
does not repair that dynamic collision by itself.

Original and world-target A/B/C/Dcomparisons are complete and retained.
The latter rejects dynamic rows on all arms; physical feasibility remains
Unknown. No extra SQP iteration, tolerance, margin, clamp or fallback is added.

## Change and acceptance

Keep the raw QP/primal, complete affine prediction array and numerical residuals
unchanged. Carry all seven exact semantic initial coordinates in a required
separate artifact field; independently replay the certified control sequence
from that state. Missing semantic state is invalid, with no fallback to rawx0.
Existing artifact validation
and full nonlinear wall/dynamic/terminal proof remain mandatory. The Stop
successor and nonlinear architecture oracle already use semantic initial
coordinates. Align the exact-row external comparison artifact too, retaining
its original raw-primal row verification before physical replay.

The first implementation overwrote affine state0. Full regression correctly
rejected35testrecords because it broke numerical progress-dynamics consistency.
That implementation is rejected and its log is retained. The separate required
semantic field keeps both numerical and physical contracts intact. Add a
course-boundary regression preserving the full raw residual sequence and
explicitly rejecting missing/outside semantic inputs.

Add failing solver-producer regression first, including latest-state feedback.
Then replace the raw physical-initial consumer, document this artifact meaning, run
focused and package tests/build, and replay original396/941before any live
trial. The complete peer-body/side-constraint audit continues independently.
No new normal authority or optional compatibility fallback is introduced.

Rollback: revert only the semantic-initial slice to5cfc50bcbehavior, retaining
the peer patch and all failed-run evidence. Integrated acceptance remains open.
