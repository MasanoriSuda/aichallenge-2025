# Stop proof provenance and publication switching

Continue autonomous full MPCC completion after local commitae6862aa.
No runtime/build currently active. Rollbackae6862aa. All442prior artifacts and
69snapshots/151experiments remain preserved. Full acceptance remains open.

Latest dev2first moving Emergency925is distinct from the atomic first terminal
world923/normal328. World923does not contain925's physical observation or the
newly selected Stop395artifact. First reconstruct the preserved artifact with
zero solves and extract same-run odometry/serialized commands. Build the
publication chronology without claiming selection logs prove publication.
Any new candidate is a separate sealed solve; never substitute it for328/395.

An independently observed producer mismatch merits a bounded native check:
stop_lattice_shadow::build_wall_snapshot uses replay.bound_tolerance_m, while
architecture_comparison::wall_snapshot uses trajectory.lateral_bound_tolerance_m.
The exact dynamic source validator requires its physical snapshot/trajectory
certificate tolerances to match. Single Stop telemetry sometimes gives invalid
dynamic proof with empty blocker; do not attribute this until native/source-copy
observation identifies SourceValidationReason. Preserve strict validation and
physical margins. Candidate proof metadata must be owned by the actual solved
trajectory; source-generation tolerances cannot stand in for post-solve values.

Inspect whether normal328was republished after923's Stop395selection and why
current Stop availability/retained proof vanished by925. D1callback3overruns
(max31.967ms) occur near1788913399.102in the9reportedwindows. Both command
streams remain40Hz/no receiptgap>50ms. Startup causal-observation warnings are
at0m/s and are not a moving-race root cause. No retry/hold/grace/timeouts/margins,
solver-budget tuning or assumed physical infeasibility. Compare architectures
on the sealed923world before another patch in its recurrent failure family.

Use failing native/replay, exact producer repair, regression/build/package,
fixed runtime acceptance and final source/artifact/proof/command identity checks.
Preserve all rejected/unknown outcomes; update registry/status and locally commit
necessary work. Continue remaining intents/dev3/dev4/gates/submission afterward.

## Demonstrated metadata defect (independent of the live 923 failure)

Native `CurrentWorldStopBindsCertificateToSolvedTrajectory` supplies a valid
pre-solve observation baseline 4.2e-5 distinct from the actual solved artifact's
residual bound. Before repair, the existing stopped, wall-clear candidate fails
dynamic proof with an empty blocker (`obstacle=/minimum=inf`). The other four
current-world Stop tests pass. The producer must bind both physical proofs to
`trajectory.lateral_bound_tolerance_m`; the strict source validator stays intact.
The rejection detail now records the existing source-validation reason, so a
provenance failure cannot masquerade as an unexplained peer collision.
No new authority, candidate, changed hard bound or configured tolerance. No
obsolete alternate authority to retire. Rollback remains ae6862aa. Acceptance:
new native regression, existing strict provenance test, package/build, frozen
replay, then single/coupled runtime. The 923/925 live root cause remains open.

The sealed 923 comparison completes the A/B/C/D architecture gate. Nine normal
arms reject (rough-right C solves but fails physical wall proof); two of four
independent stop-support comparisons accept. These are different candidate
problems from the current production free-controls candidate (which rejects
before proof). Capture their exact source/artifact semantics before drawing
producer conclusions. Physical infeasibility is not established.
