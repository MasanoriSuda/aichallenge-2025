# Peer observation epochs and derived Stop evidence

Continue autonomous completion after 36c2a01a. Current production is fixed.
Exact same-376 Accepted 928 / terminal-rejected 929 in publication-clock-dev2-r1
has equivalent publication clock mapping, now repaired. Unlike earlier pairs,
peer-only cross-substitution reverses both outcomes. Old 928 field:
now 9.789999781, generation 161, d2 position (89631.89925680718,43132.49021134979),
velocity (-1.1513923493164182,1.0248150202914346). New 929 field:
now 9.81999978, generation 162, position (89631.85018814907,43132.52488509557),
velocity (-1.2405925467431673,1.048942278501929). Radius unchanged 1.9309999998882412.

Read the same run's raw V2X, source/array stamps, bag receipt clocks, D2 state
and exact controller tracker. Distinguish a real motion update, filter/model
approximation and a proved state/epoch producer defect. Filter lag alone is
not automatically a bug. Do not tune gains/noise/radius/delays/solver limits
or obtain body heading from V2X velocity. Wire covariance is metres standard
deviation. Local GNSS lever arm and EKF clock repairs remain unchanged.

D1 929 independent Stop is accepted and derived 385 joins. First Emergency
930 examines 385, but recorder requires solver_source_snapshot before writing
its existing execution_artifact. Actual-publication evidence is missing too.
Do not replace 385 with a newly solved 376 or treat ordinary Accepted 928 as
same-artifact evidence. Inspect actual derived artifact and physical proof,
then reproduce the missing observation with a native test before a minimal
recorder change. Parent derivation provenance must be distinguished from a
solver-source certificate; no fabricated source or additional control authority.

Ego observation-age mismatch remains separately recorded in the preceding
slice. Before a production fix: earliest invariant, failing native/replay,
producer, obsolete mask, acceptance and rollback. Compare sealed-world A/B/C/D
before another formulation patch. All-method failure remains Unknown.
Finish appropriate tests/build/runtime, timing, sealing/registry/spec/status
and local commit; then continue coupled/intents/dev3/dev4/gates/submission.

Selected observation repair: attach the immutable CertifiedPlan at the existing
final-publication observation boundary. The worker writes a separate additive
mpcc-certified-plan-observation/v1 node for published, inspected and previous
inspected roles. It contains complete artifact context/course frame, exact
physical wall-proof source, diagnostic and independently owned grid payload.
Legacy solver-source status stays missing when absent; no fake source, parent
solve, new authority or certificate relaxation. Current dynamic revalidation
inputs remain separately recorded; this node does not invent the original
parent's dynamic proof or solve provenance. The same atomic directory publish
and finite first-boundary FIFO remain in use.
